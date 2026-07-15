# Session Summary — Pendulum Codebase Analysis, Automated Experiment, and System ID

This document recaps everything done in this session, in the order it happened, what was delivered, where it lives, and what's still worth double-checking.

Repo analyzed: `github.com/Mohan-afk/Pendulum` (a RobStride RS00 rotary inverted pendulum, controlled over CAN via a USB-to-CAN adapter). Local working folder on your machine: `C:\Users\manan\Downloads\Robostride Motor\Pendulum\Manan`.

---

## 1. Codebase architecture analysis

**Ask:** analyze the codebase thoroughly, understand its architecture/control flow/APIs, and recommend improvements for future ROS integration, advanced motor-control work, and system identification.

**What I did:**

- Cloned the GitHub repo and read every file: `README.md`, `docs/protocol_notes.md`, `Manan/FUNCTIONS.md`, all of `Manan/*.py`, all of `src/*.py`, `.gitignore`, and the committed `Manan/zero_calibration_log.json`.
- Mapped the hardware/software stack: RS00 actuator → RobStride USB-to-CAN adapter (a serial port wrapping real CAN frames in a proprietary "AT" envelope, 921600 baud, 1 Mbps CAN) → `ATCanBus` driver → protocol helpers (`enable`, `stop`, `control`, `decode_feedback`, `make_id`, fixed-point packing) → the actual experiment scripts.
- Found that the repo contains **two parallel generations of the same code**: `src/` (what the README documents as primary) and `Manan/` (a cleaner rewrite with tests, docs, and argparse CLIs, but never linked from the README). The protocol implementation is duplicated **four separate times** across the two.
- Found and verified concrete bugs/inconsistencies by reading the actual code (not just describing them):
  - **Motor CAN ID mismatch**: `docs/protocol_notes.md` and `Manan/FUNCTIONS.md` both document `motor1 = 1`, but the real working scripts (`src/inverted_pendulum.py`, `src/soft_balance.py`) and the one real hardware session recorded in `zero_calibration_log.json` all use `motor_id = 127`.
  - **Crash risk in `src/`**: `src/at_can_bus.py`'s `recv()` doesn't validate frame length before slicing, and `decode_feedback()` unpacks with no length check — a truncated serial read can raise `struct.error` and kill the control loop mid-swing. `Manan/at_can_bus.py` already fixes both, but the README-documented scripts use the older, unpatched version.
  - `Manan/__pycache__/at_can_bus.cpython-314.pyc` and `Manan/zero_calibration_log.json` (a real hardware session's data) are both committed to git despite `.gitignore` listing `__pycache__/`.
  - Rod mass/length constants (`ROD_MASS_KG`, `ROD_LENGTH_M`) are copy-pasted separately in two files instead of centralized.
- Wrote a full recommendations report covering project structure (consolidating to one installable package), abstractions (a `Motor` class, pure-function control laws separate from I/O — the specific change that makes ROS integration and swapping control laws tractable), what's needed for system identification specifically, testing, logging, and documentation/workflow gaps (no CI, no lint config, no type hints).

**Delivered:** `ARCHITECTURE_AND_ROADMAP.md` — sent via chat only (not written to your local folder, since it covers the whole repo, not just `Manan/`).

---

## 2. Automated free-release experiment script

**Ask:** build an automated experiment that, for each target angle from 3° to 90° in 3° steps (5 trials each), moves the pendulum to the angle, zeroes all control outputs for a true free release, and logs position/velocity/torque for exactly 5 seconds per trial.

**What I did:**

- Flagged upfront that this cloud session has no serial/USB access to your actual motor — I could only write and validate the script, not run it against real hardware myself.
- Confirmed via the device bridge that your local `Manan/at_can_bus.py` is byte-identical to the GitHub version (just Windows line endings), and confirmed from `zero_calibration_log.json` that the real hardware config is `motor_id=127` / `COM8`.
- Asked you to confirm the angle convention (measured from calibrated zero / hanging straight down) and where the script should be delivered, before writing 150 trials' worth of hardware-moving code.
- Built `pendulum_release_experiment.py` on top of the canonical `Manan/at_can_bus.py` driver, with a per-trial state machine: smoothstep-ramp to target under position hold → wait for **genuine** velocity settle (not a blind delay) → zero every control output (kp=kd=torque=0, true free release) → log timestamp/position/velocity/torque/temp as fast as the request/response protocol allows for exactly 5.0s (timed off a monotonic clock) → brake (damping-only frames) before the next trial.
- Safety/robustness features: a pre-flight feedback check before starting, a loud clearance confirmation prompt, incremental CSV writes with `flush()` after every row (a crash mid-sweep doesn't lose completed trials), Ctrl+C triggering the same brake→stop→shutdown sequence as a clean exit, and a `--quick-test` mode for a fast 2-trial sanity check before committing to the full ~20-minute sweep.
- Validated the entire control flow **offline**, against a simulated pendulum (a small physics stub standing in for the real motor), before ever sending it to you — this caught issues like numpy API changes and confirmed the file-output structure was correct without needing your hardware.
- **Follow-up edit:** you asked for every trial to explicitly start at position 0. I added a "return to start position and settle" phase at the top of every trial (before the ramp to that trial's target), exposed as `--start-pos-deg` (default `0.0`), and recorded the actual settled start position/velocity per trial in the metadata JSON. Re-validated with the same offline harness — confirmed every trial genuinely settles within a fraction of a degree of 0° before ramping to target.

**Delivered:** `pendulum_release_experiment.py`, sent via chat and written to your `Manan` folder via the device bridge.

> **Note — please double-check this:** the *first* version (without the "return to 0°" step) was successfully written to your `Manan` folder. When I sent the *updated* version (with `--start-pos-deg` / the return-to-zero step), the device bridge first timed out and then reported your desktop app as disconnected, so that final commit did **not** land on disk automatically — only the outdated first version may still be sitting in your folder. The correct, final version was delivered in chat; please confirm the copy in `Manan/pendulum_release_experiment.py` on your machine matches what I sent (it should contain a `--start-pos-deg` argument and a "Returning to start position" print statement) before running it.

---

## 3. Running the experiment (on your side)

You ran `pendulum_release_experiment.py` on your machine against the real hardware and uploaded the results: `pendulum_release_sysid_20260715_211042.csv` (54,312 rows) and its `_meta.json` sidecar — the full 30-angle × 5-trial sweep.

---

## 4. Offline system identification analysis

**Ask:** write code to analyze the collected data for dry friction and damping.

**What I did:**

- Loaded and explored the real data before deciding on a method, rather than assuming the textbook "count oscillation peaks" approach would apply. That exploration turned up something important: **releases at 3°–30° essentially never move at all** (static friction alone holds the pendulum against gravity), releases at **33° and above always swing through vertical**, and **every trial settles at a nonzero rest angle** (up to ±16° from vertical) rather than returning to the calibrated zero. This is a real static-friction "dead zone" in the hardware, not a data artifact — confirmed by checking the raw position traces directly.
- Because most trials never complete even one full oscillation, the standard peak-decay method wouldn't have enough data. Instead, I used an **energy-dissipation regression** as the primary method: every trial starts and ends at rest, so the potential energy given up between release and final rest must equal the total work done against friction — computed directly from measured arc length traveled (Coulomb term) and ∫velocity²dt (viscous term), with no differentiation or oscillation-counting needed. This turns all 150 trials into a single linear regression.
- Cross-checked that result with a secondary nonlinear ODE trajectory fit on the 20 largest-swing trials, and discovered (and explicitly flagged) that per-trial viscous/Coulomb splits from that method are only weakly identifiable on their own (correlation −0.97 between the two fitted parameters across trials) — which is exactly why the pooled energy-regression method, using the full range of release amplitudes, is the more trustworthy one for separating the two effects.
- Generated six diagnostic plots (trajectory overlays, rest-angle-vs-release-angle showing the dead zone, the energy-regression fit quality, phase portraits, ODE-fit examples, and ODE-parameter stability vs. angle) and a results summary.

**Headline results:**

| Quantity | Value |
|---|---|
| Static-friction dead zone (lower bound) | ±16.3° |
| Releases that never swing through vertical | ≤ 30° (always cross at ≥ 33°) |
| μ_c — Coulomb friction / peak gravity torque (dimensionless) | 0.230 (95% CI 0.226–0.233) |
| μ_b — viscous coefficient / peak gravity torque (seconds) | 0.0216 s (95% CI 0.0199–0.0231) |
| Energy-regression fit quality | R² = 0.9995 |
| c (Coulomb torque), using unverified reference rod params | ≈ 0.10 Nm |
| b (viscous coefficient), using unverified reference rod params | ≈ 0.0095 Nm·s/rad |
| ODE cross-check natural frequency ω₀ | ≈ 3.82 rad/s |

**Delivered:** `analyze_pendulum_sysid.py` (reusable — takes `--csv`/`--meta` args for future sweeps), sent via chat and written to your `Manan` folder; plus, sent via chat only (not written to disk): six PNG plots, `sysid_summary.json`, `trial_features.csv`, and `ode_trajectory_fits.csv`.

---

## Where things stand on your machine right now

Confirmed written to `Manan/` via the device bridge:
- `pendulum_release_experiment.py` — **likely the outdated version**, see the note in section 2 above
- `analyze_pendulum_sysid.py` — current version, confirmed written successfully

Sent in chat only (download and place yourself if you want them on disk):
- `ARCHITECTURE_AND_ROADMAP.md`
- The updated `pendulum_release_experiment.py` (with the `--start-pos-deg` return-to-zero step)
- All six analysis plots, `sysid_summary.json`, `trial_features.csv`, `ode_trajectory_fits.csv`

**One important caveat that applies to all of the physical-unit numbers above:** the rod mass (0.18 kg) and length (0.5 m) used anywhere in this session are carried over from `src/soft_balance.py` as an unverified "reference" value — never independently re-measured. The dimensionless ratios (μ_c, μ_b, ω₀) don't depend on that assumption and are safe to trust as-is; anything in Nm or kg·m² does, and should be recomputed once you've confirmed the actual rod parameters.
