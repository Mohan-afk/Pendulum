# Pendulum System Identification — Full Methodology

This is the detailed version: every modeling decision, the physics behind each equation, why the "obvious" method didn't work on this data, how the fitting actually happens, what the numbers mean, and the complete code. Nothing summarized away.

---

## 1. What the raw data actually is

`pendulum_release_experiment.py` produced 150 trials: 30 release angles (3°, 6°, 9°, ..., 90°) × 5 trials each. Each trial's row in the CSV is one polled sample during a 5-second free-release window:

```
angle_target_deg, trial, t_release_s, timestamp_unix, pos_rad, pos_deg, vel_rad_s, torque_Nm, temp_C
```

`t_release_s` is seconds since that trial's release moment (each trial restarts at ~0), `pos_rad`/`vel_rad_s` are the RS00's own encoder feedback (position and velocity are both hardware-reported, not derived from each other), and `torque_Nm` is the motor's reported torque — which should read ~0 throughout, since the whole point of a trial is that kp=kd=torque_ff=0 the instant release begins. 54,312 total rows.

Two facts about the experimental protocol matter for everything that follows:

1. **Every trial starts at true rest.** The updated experiment script forces the pendulum back to the calibrated zero (0°) and waits for a genuine velocity-settle (not a fixed delay) before ramping up to that trial's target angle, holds there under position control (kp=15, kd=1.5) until settled again, and *only then* zeros all outputs. So at `t_release_s = 0`, `omega ≈ 0` to within the settle tolerance (`--settle-vel-thresh 0.05 rad/s`).
2. **Every trial ends at rest too**, just not necessarily where you'd expect — see below.

## 2. Exploratory analysis first — and why it changed the plan

I did not walk in assuming a method. The first thing I did was load the data and look at raw trajectories before picking an analysis approach, because assuming "it'll oscillate a few times and decay" without checking is how you end up fitting noise.

Plotting `pos_deg` vs `t_release_s` for a handful of trials revealed two things immediately:

- **Small releases don't move.** Below a target of ~30°, the pendulum released from that angle just... stays there (modulo small noise), never swinging back toward vertical at all within the 5-second window.
- **Large releases don't return to vertical either.** Even a 90° release, swinging hard through the bottom, settles at some nonzero final angle — not 0°, and not the same angle from trial to trial.

I quantified this with `trial_features()` and `analyze_dead_zone()` (code in §7). For every (angle, trial) pair I computed `theta_final` = mean of the last 15 logged samples (a robust "did it come to rest, and where" estimate — better than the very last raw sample, which can be noisy), and a boolean `crossed_zero` flag (did `theta` ever cross 0° during the 5s window, ignoring sub-0.5° noise so a stationary trial sitting exactly on 0° isn't miscounted).

Results, straight from `sysid_summary.json`:

```
max_release_angle_deg_with_no_crossing: 30.0
min_release_angle_deg_that_always_crosses: 33.0
dead_zone_half_width_deg_lower_bound: 16.28
n_trials_never_crossing: 50 / 150
```

So there's a sharp transition between 30° and 33° — below it, nothing crosses vertical; above it, everything does — and across *all* 150 trials, the largest final rest angle magnitude observed was 16.28°. That's not measurement noise; it's **static friction (stiction) large enough to hold the pendulum in equilibrium anywhere within roughly ±16° of true vertical**, not just exactly at the bottom. Gravity's restoring torque at small angles (`∝ sin(theta)`) is simply too weak to overcome static friction near the bottom of the swing, so the system has a whole *band* of angles where it can stay put, not a single equilibrium point.

This is a textbook **Coulomb / dry-friction dead zone**, and it's a real physical property of this rig (gearbox + bearing stiction in the RS00's quasi-direct-drive train), not an artifact of the analysis.

### Why this kills the "obvious" method

The standard way to identify friction and damping from a free-oscillation release is **peak-envelope decay**: let the pendulum swing, find each successive peak `theta_1, theta_2, theta_3, ...` (local extrema between zero-velocity crossings), and look at how the peak amplitudes shrink.

- Pure viscous damping → **exponential decay**: `theta_n ≈ theta_0 * exp(-zeta * n)`.
- Pure Coulomb (dry) friction → **constant absolute decrement per half-swing**: `theta_n ≈ theta_0 - n * delta`, where `delta` is a fixed angle lost to friction each swing.
- Both together → some combination of the two, usually separated by fitting `delta_n = theta_n - theta_{n+1}` as a function of `theta_n` (a straight line: dry-friction intercept + viscous-slope term).

That method needs *several completed oscillation cycles* per trial to get enough `(theta_n, delta_n)` pairs to regress on. Here, 50 of 150 trials never even complete a quarter-cycle (they don't cross vertical at all), and even the ones that do swing through mostly settle into the dead zone after one or two swings rather than oscillating cleanly for many cycles. There just isn't a usable peak sequence for most of the dataset. Trying to force peak-decay fitting here would mean throwing away a third of the data outright and fitting noisy 2-3-point "decay curves" on much of the rest — not something I was willing to present as a real result.

## 3. The method I actually used: energy-dissipation regression

### 3.1 The physical idea

Instead of tracking the shape of the decay, use the fact that **every trial is a complete rest-to-rest event**. At release (`t=0`), the pendulum is at `theta_0` with `omega ≈ 0`. By the end of the 5-second window, it's settled at `theta_final` with `omega ≈ 0` again (confirmed in the code by looking at the tail — see `omega_tail_std` in `trial_features()`, which stays small for essentially every trial). Kinetic energy is (approximately) zero at both endpoints, so **all of the gravitational potential energy released between those two configurations was dissipated by friction along the way** — no other energy sink exists (motor commands are all zero, so no electrical work is being done on the system).

Write the pendulum's potential energy relative to the pivot, treating it as a rigid body with center of mass at distance `L/2` from the pivot:

```
U(theta) = -m * g * (L/2) * cos(theta)          [taking straight-down as theta = 0]
```

So the energy released between `theta_0` and `theta_final` is:

```
Delta_U = U(theta_0) - U(theta_final)
        = -k_g * cos(theta_0) - (-k_g * cos(theta_final))
        = k_g * [cos(theta_final) - cos(theta_0)]
```

where `k_g := m * g * (L/2)` is the peak gravity torque (the torque gravity exerts at theta = 90°, i.e. horizontal). Careful with the sign here: if the pendulum starts higher (`theta_0` far from 0) and ends lower/closer to vertical (`theta_final` near 0), then `cos(theta_final) > cos(theta_0)`, so `Delta_U` as defined is **positive** — energy *was* released and had to go somewhere. Good, that matches physical intuition.

Now the dissipative side. I modeled friction as the standard combination of Coulomb (dry) and viscous terms acting on the pendulum's equation of motion:

```
I * theta'' = -k_g * sin(theta) - b * theta' - c * sign(theta')
```

`c` is the Coulomb friction torque (constant magnitude, opposes velocity direction), `b` is the viscous damping coefficient (opposes velocity, magnitude proportional to speed). The work done *against* friction over a trajectory is the negative of the work friction does *on* the pendulum, integrated along the path:

- **Coulomb term**: friction torque has constant magnitude `c`, always opposing motion, so the work it removes per unit angle traveled is just `c` times the angle traveled — regardless of speed or direction reversals. Over the whole trial, total energy removed by Coulomb friction is `c * S`, where `S = integral of |theta'| dt = total arc length traveled` (in radians) — this is why I compute `arc_length_rad` as `sum(|diff(theta)|)` on the position trace directly: it's robust to however many times the pendulum reverses direction, no oscillation-counting needed.
- **Viscous term**: instantaneous power dissipated by a viscous term is `b * theta'^2` (force times velocity, with velocity-proportional force), so total energy removed is `b * integral(theta'^2 dt) =: b * V`. I compute `V` (`visc_integral` in the code) as the trapezoidal integral of `omega^2` over the trial's time series, using the *measured* velocity channel directly (not a finite difference of position — the RS00 reports velocity as its own encoder-derived quantity, so this avoids amplifying position noise through differentiation).

Energy balance (energy released by gravity = energy consumed by both friction mechanisms):

```
k_g * [cos(theta_final) - cos(theta_0)]  =  c * S  +  b * V
```

This holds **per trial**, exactly, under the model — and critically, it requires no oscillation counting, no peak-finding, no assumption about how many times the pendulum swings back and forth. It works identically whether the trial swings zero times (dead-zone trials, where `S` is tiny and `X` is tiny) or many times.

### 3.2 Making the regression not depend on an unverified mass

`k_g = m * g * (L/2)` depends on the rod mass and length, and those values (`0.18 kg`, `0.5 m`) were never independently re-measured for this rig — they're carried over from `src/soft_balance.py` as a "last known value" comment in the original code, which I flagged as a real risk during the architecture review and am flagging again here. I didn't want the headline friction numbers to silently inherit that uncertainty.

So I divide the whole equation by `k_g`:

```
X := cos(theta_final) - cos(theta_0)   =   mu_c * S   +   mu_b * V
```

where `mu_c := c / k_g` (dimensionless — Coulomb torque as a fraction of peak gravity torque) and `mu_b := b / k_g` (units of seconds — viscous coefficient as a fraction of peak gravity torque). **These two ratios are estimated with zero dependence on the rod-mass assumption.** Only if you want `c` and `b` in physical Nm units do you need to multiply back through by a `k_g` you trust.

### 3.3 Fitting

This is now a plain 2-parameter linear regression, no intercept (the physics has no free constant — `X=0` exactly when `S=V=0`, i.e. a trial that doesn't move at all releases zero energy, trivially true):

```python
A = np.column_stack([S, V])              # (150, 2) design matrix
coef, *_ = np.linalg.lstsq(A, X, rcond=None)
mu_c, mu_b = coef
```

Fit quality is checked two ways:

1. **R²** on the fit itself.
2. **A second fit *with* a free intercept**, purely as a physical sanity check — the model predicts the intercept should come out ≈ 0 (since a trial with no motion truly should dissipate no energy), and if it came out large that would be a red flag that something in the energy-balance derivation was wrong (e.g., an unaccounted energy source, wrong sign convention, or an offset in the angle-zero calibration). This is not used for the reported `mu_c`/`mu_b` — those come from the no-intercept fit — it's purely a diagnostic.

**Confidence intervals**: 2000-resample bootstrap — resample the 150 trials with replacement, refit `(mu_c, mu_b)` each time, and take the 2.5th/97.5th percentiles of the resulting distributions. This captures fit uncertainty without assuming Gaussian residuals or needing a closed-form OLS covariance formula (which would be fine here too, but bootstrap is a strictly weaker set of assumptions and cheap to run — 2000 refits of a 150×2 lstsq is milliseconds).

### 3.4 Results

```
mu_c = 0.2295   (95% CI: 0.2260 – 0.2331)
mu_b = 0.0216 s (95% CI: 0.0199 – 0.0231)
R²   = 0.9995
intercept check = -0.0024   (≈ 0, as predicted — good sign)
n trials used    = 150 (all of them, including the 50 that never crossed zero)
```

An R² of 0.9995 across all 150 trials — including the "boring" dead-zone trials that barely move at all — is a strong signal that the linear energy-balance model is capturing the real dominant physics, not overfitting: those near-stationary trials have small `S`, small `V`, and small `X` all consistently near the origin, which is exactly what the model predicts and what pulls R² up rather than down (a model that was wrong wouldn't automatically nail the near-zero-motion trials too).

Converting to Nm units using the **unverified** reference `k_g = 0.18 * 9.81 * (0.5/2) = 0.44145 Nm`:

```
c ≈ 0.1013 Nm      (Coulomb / dry friction torque)
b ≈ 0.00954 Nm·s/rad   (viscous damping coefficient)
```

These two are the ones to recompute if/when the rod mass and length get independently measured — everything upstream of them (`mu_c`, `mu_b`, the CIs, R²) is safe regardless.

## 4. Cross-check: nonlinear ODE trajectory fitting

The energy method is elegant but it only uses two scalar summaries per trial (`S` and `V`) — it throws away the trajectory *shape*. As a cross-check, I independently fit the full nonlinear equation of motion to the swinging trials' actual position-vs-time curves and see whether the resulting friction/damping picture agrees.

### 4.1 The model

```
theta'' = -omega0^2 * sin(theta)  -  zeta_v * theta'  -  a_c * tanh(theta' / eps)
```

This is the same physical equation of motion as before, but reparametrized as "per unit inertia" quantities so I don't need to assume `I` to fit it: `omega0^2 := k_g/I` (natural frequency squared — this is the standard nonlinear-pendulum ODE for the gravity term, `theta'' = -omega0^2 sin(theta)` for the frictionless case, which is why fitting `omega0` alone, independent of friction, is a nice extra cross-check of internal consistency), `zeta_v := b/I` (viscous rate), and `a_c := c/I` (a Coulomb term expressed as an angular deceleration magnitude, `eps=0.02 rad/s` a small smoothing width).

**Why `tanh(theta'/eps)` instead of `sign(theta')`:** true Coulomb friction is a sharp discontinuity at zero velocity, and gradient-based nonlinear optimizers (`scipy.optimize.least_squares`, which needs to numerically differentiate through the ODE solve to find a descent direction) behave badly around discontinuities — the solver can get stuck or take pathological steps right as the pendulum crosses zero velocity, which happens exactly at every peak of every swing, i.e. constantly. `tanh(theta'/eps)` is a smooth approximation that's essentially indistinguishable from `sign()` for `|theta'| >> eps` (0.02 rad/s is tiny compared to typical swing velocities of several rad/s) but differentiable everywhere, so the optimizer stays well-behaved.

### 4.2 Fitting procedure

For each selected trial: simulate the ODE forward from the trial's actual `(theta_0, omega_0)` using `scipy.integrate.solve_ivp`, compare the simulated `theta(t)` against the measured `theta(t)`, and let `scipy.optimize.least_squares` adjust `(omega0², zeta_v, a_c)` to minimize the sum of squared position residuals, with bounds `omega0² ∈ [0.1, 200]`, `zeta_v ∈ [0, 20]`, `a_c ∈ [0, 50]` to keep the solver in a physically sane region and away from degenerate solutions.

This is done only for trials with release angle ≥ the auto-detected swing threshold (33°, from Finding 1 — no point fitting a swing model to a trial that didn't swing), and only 1 trial per angle by default (20 total fits) rather than all 100 swinging trials, because each fit requires an iterative nonlinear optimizer that itself calls a numerical ODE solver at every trial step — orders of magnitude slower than the closed-form linear-algebra energy fit. I explicitly tuned this for runtime (see §4.3) rather than silently truncating the dataset without saying so; the `--ode-trials-per-angle` flag exposes the tradeoff if a more thorough (slower) cross-check is wanted later.

### 4.3 Performance tuning (this genuinely mattered)

My first version of this fit used the full dataset (100 swinging trials, all trials per angle), the default `RK45` solver at tight tolerances (`rtol=1e-6, atol=1e-8`), and `least_squares`'s default `max_nfev` (2000 function evaluations per fit). Profiling showed **~2.76 seconds per trial fit**, which times 100 trials blew well past a reasonable runtime. I made four changes together:

1. Reduced to 1 trial per angle by default (100 → 20 fits) — exposed as `--ode-trials-per-angle` for anyone who wants the full sweep later.
2. Downsampled each trial's time series by a stride of 3 before fitting (`downsample_stride=3`) — the position trace doesn't need every single polled sample to constrain a 3-parameter ODE fit well.
3. Switched the solver from `RK45` to `RK23` with looser tolerances (`rtol=1e-4, atol=1e-6`) — a lower-order adaptive method that's noticeably cheaper per step and plenty accurate for this smooth, non-stiff, second-order pendulum ODE.
4. Capped `least_squares` at `max_nfev=150` instead of the 2000 default — the residual landscape here is well-behaved enough (3 parameters, decent initial guess) that it converges well before 150 evaluations in practice; this just prevents pathological trials from stalling the whole run.

End result: full script runtime dropped to about 20 seconds. All four changes are visible in `_simulate()` and `fit_ode_trajectories()`'s function signature defaults in the code below.

### 4.4 Results and the identifiability problem I found (and didn't hide)

```
n trials fit                = 20
omega0 (median)              = 3.818 rad/s
zeta_v (median)               = 0.671 /s
a_c (median)                   = 2.777 rad/s²
median trajectory fit RMSE     = 1.33°
```

`omega0` agrees well internally (it's derived independently of any friction assumption, purely from oscillation frequency, and gives an implied inertia `I = k_g / omega0² ≈ 0.0303 kg·m²` using the reference `k_g` — a useful independent cross-check quantity even without trusting the friction split).

But when I looked at the **per-trial** `(zeta_v, a_c)` pairs individually (visible in `06_ode_param_stability.png`), they were noticeably more scattered across release angle than I expected for a "should be roughly constant" physical parameter — and checking the correlation between the two fitted parameters across the 20 trials came out to **-0.97**. That's a near-perfect negative correlation: whenever one trial's fit assigns more damping to the viscous term, it assigns correspondingly less to the Coulomb term, and vice versa, almost exactly compensating.

That's a real **parameter identifiability problem**, not noise, and it makes physical sense once you think about it: a single roughly-one-swing trajectory doesn't give the optimizer much leverage to distinguish "this decayed because of speed-proportional drag" from "this decayed because of a constant friction torque" — both reduce swing amplitude in a broadly similar way over one or two cycles, so the optimizer can trade one off against the other while barely changing the fit residual. You'd need either many oscillation cycles per trial (to see the *shape* of the decay curve, which does differ between the two mechanisms — exponential vs. linear) or, as it happens, **many trials at different amplitudes** — which is exactly what the energy-dissipation method (Finding 2) uses, since Coulomb dissipation scales with distance traveled while viscous dissipation scales with the *square* of velocity integrated over time, and those two quantities decorrelate across a wide range of release amplitudes even when a single trial can't separate them.

I reported this explicitly rather than quietly picking one method's numbers: the ODE cross-check's `omega0` and the overall RMSE (1.33°, a good trajectory fit) are trustworthy, but its per-trial `(zeta_v, a_c)` split is not — which is exactly why Finding 2's pooled regression, not Finding 3, is the one I'm calling primary for the `b` vs `c` split.

As a further cross-check, converting the ODE fit's median `(zeta_v, a_c)` to Nm units via the *ODE-implied* inertia (rather than the energy method's `k_g`-normalized ratios) gives `b ≈ 0.0203 Nm·s/rad`, `c ≈ 0.0841 Nm` — same order of magnitude as Finding 2's `b ≈ 0.0095`, `c ≈ 0.1013`, agreeing on Coulomb friction being the dominant dissipation mechanism (both put `c` roughly 5-10x `b` in relative contribution at typical swing speeds), even though the exact per-trial split is the part known to be shaky.

## 5. Software choices and why

- **`np.linalg.lstsq`** for the linear regression rather than `sklearn` or `statsmodels` — it's a 2-parameter, no-intercept, well-conditioned linear least squares; pulling in a whole modeling library for this would be needless dependency weight for something numpy solves directly and transparently.
- **Bootstrap CI over analytic OLS CI** — fewer distributional assumptions, and cheap enough (2000 resamples of a 150×2 system) that there's no real cost to being more conservative here.
- **`scipy.integrate.solve_ivp` + `scipy.optimize.least_squares`** for the ODE cross-check — standard, well-tested numerical tooling; the smoothing/tolerance/`max_nfev` choices are explained in §4.3.
- **`groupby(...).groupby('angle_target_deg', group_keys=False).head(trials_per_angle)`** for selecting which trials go into the ODE cross-check, instead of a `.apply()`-based "top N" — this was actually a bug-fix: pandas 3.0's `groupby().apply()` dropped support for the `include_groups` kwarg I'd originally used, and the `sort_values(...).groupby(...).head(...)` approach is both simpler and avoids the whole `apply` deprecation surface.
- **`_trapz = getattr(np, 'trapezoid', None) or np.trapz`** — numpy ≥2.0 renamed `trapz` to `trapezoid`; this shim keeps the script running on whatever numpy version is installed without hardcoding either name.
- **Monotonic-safe, no wall-clock dependence in the analysis itself** — this only matters in the experiment script, not here, but worth noting the two scripts share that discipline: the CSV's `t_release_s` was already generated from `time.monotonic()` deltas in the experiment script, so no NTP jump or clock-adjustment artifacts can appear in the analysis's time axis.
- **Plots use `viridis`**, a single sequential colormap keyed to release angle throughout (not a rainbow map, not dual y-axes) — release angle is a continuous ordered variable, and viridis is perceptually uniform and colorblind-safe, which matters here since the whole point of several plots (trajectory overlay, phase portraits, energy-fit parity) is to visually track a trend *across* that continuous variable.
- **Every plot was actually opened and visually inspected** (via the `Read` tool on the generated PNGs) before I called the analysis done — confirming axis labels were legible, the colorbar rendered, the dead-zone shading appeared where expected, and the parity plot's points actually clustered near the `y=x` line as the R²=0.9995 claimed, rather than trusting the code ran without checking what it produced.

## 6. Caveats — read before trusting numbers downstream

- **Rod mass (0.18 kg) and length (0.5 m)** are carried over from `src/soft_balance.py` as a last-known/reference value, never independently re-measured on this rig. `mu_c`, `mu_b`, `omega0`, R², and the CIs do not depend on this assumption at all and are safe as-is. Anything expressed in Nm (`c`, `b`) or kg·m² (`I`) does depend on it and should be recomputed once the rod is actually weighed/measured.
- **Per-trial `(zeta_v, a_c)` from the ODE cross-check are not individually trustworthy** (§4.4) — use the median/`omega0` from that method, and prefer the energy-dissipation regression for the `b` vs `c` split specifically.
- **The dead-zone half-width (±16.28°) is a lower bound**, not a precise boundary — it's simply the largest `|theta_final|` actually observed across 150 trials; the true stiction boundary could be marginally larger and this experiment wouldn't know, since no trial happened to release from an angle/energy combination that would have revealed a wider dead zone.
- **Temperature effects are unexamined.** `temp_C` was logged but not used in this analysis; stiction and viscous coefficients in real actuators can drift with winding/gearbox temperature over a ~20-minute sweep. Worth checking if results seem inconsistent across the early vs. late trials of the run.

## 7. Complete source code

Verbatim contents of `analyze_pendulum_sysid.py` (the version that produced every number above):

```python
#!/usr/bin/env python3
"""
analyze_pendulum_sysid.py

Offline system identification of dry (Coulomb/static) friction and viscous
damping from the automated free-release angle sweep produced by
pendulum_release_experiment.py (angle_target_deg, trial, t_release_s,
timestamp_unix, pos_rad, pos_deg, vel_rad_s, torque_Nm, temp_C).

WHY THIS METHOD (read this before trusting the numbers)
---------------------------------------------------------------------------
The classic approach for this kind of experiment is "peak-envelope decay":
find successive oscillation peaks after release and see how fast their
amplitude shrinks (exponential decay -> viscous; constant absolute
decrement each half-swing -> Coulomb). That method needs the pendulum to
actually oscillate through several cycles.

This particular dataset does not do that. Releases below ~30 degrees barely
move at all (static friction alone holds them against gravity), and *every*
trial -- even the big releases -- comes to rest at a nonzero angle, not back
at the calibrated zero. That is itself a real, useful finding (a sizeable
static-friction "dead zone"), but it also means peak-counting has almost no
data to work with: most trials never complete even one full swing back past
vertical.

So the primary method here is an ENERGY-DISSIPATION regression instead:
every trial starts at rest (theta_dot ~ 0, confirmed by the experiment
script's settle check) and ends at rest, so ALL of the potential energy
given up between release and final rest was dissipated by friction --
no oscillation counting required, and it uses every trial, including the
ones that don't move. See fit_energy_dissipation() docstring for the math.
A secondary ODE trajectory-fit method (fit_ode_trajectories()) is used as a
cross-check on the subset of trials that do swing through vertical.

Because the actual rod mass/length were never independently re-measured for
this rig (only carried over as an unverified "reference" from
src/soft_balance.py), the headline outputs are the DIMENSIONLESS ratios
mu_c = c/k_g (Coulomb friction torque / peak gravity torque) and
mu_b = b/k_g (viscous coefficient / peak gravity torque, units of seconds)
-- these don't depend on trusting the reference mass at all. Nm-scale
(c, b) and an inertia estimate are also reported, clearly marked as
depending on that unverified reference.

Usage:
    python analyze_pendulum_sysid.py --csv data/pendulum_release_sysid.csv \\
        --meta data/pendulum_release_sysid_meta.json --out-dir out/
"""
import argparse
import json
import os

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import integrate, optimize

G = 9.81

# numpy >=2.0 renamed trapz -> trapezoid; support either so this runs on
# whatever numpy the user has installed.
_trapz = getattr(np, 'trapezoid', None) or np.trapz


# ---------------------------------------------------------------------------
# Loading & per-trial feature extraction
# ---------------------------------------------------------------------------

def load(csv_path, meta_path):
    df = pd.read_csv(csv_path)
    df = df.sort_values(['angle_target_deg', 'trial', 't_release_s']).reset_index(drop=True)
    with open(meta_path) as f:
        meta = json.load(f)
    return df, meta


def trial_features(df):
    """One row per (angle, trial) with the quantities the fits below need.
    Everything here comes straight from measured position/velocity -- no
    differentiation, no assumed physical parameters."""
    rows = []
    for (angle, trial), g in df.groupby(['angle_target_deg', 'trial']):
        g = g.sort_values('t_release_s')
        t = g['t_release_s'].to_numpy()
        theta = g['pos_rad'].to_numpy()
        omega = g['vel_rad_s'].to_numpy()

        theta0 = theta[0]
        # final rest position/velocity: mean of the last 15 samples (or fewer
        # if a trial logged less than that)
        tail_n = min(15, len(theta))
        theta_final = float(np.mean(theta[-tail_n:]))
        omega_tail_std = float(np.std(omega[-tail_n:]))

        # total arc length traveled (robust to oscillation, no assumption of
        # a fixed period): sum of |consecutive position deltas|
        arc_length = float(np.sum(np.abs(np.diff(theta))))

        # viscous dissipation integral: trapezoidal integral of omega^2 dt,
        # using the *measured* velocity (not a finite-difference of theta)
        visc_integral = float(_trapz(omega ** 2, t))

        crossed_zero = bool(np.any(np.sign(theta - 0.0) != np.sign(theta0)) and
                             np.any(np.abs(theta) > 0.5 * np.pi / 180))  # ignore sub-0.5deg noise

        # count velocity zero-crossings (oscillation half-cycles) as a
        # secondary diagnostic
        sign_changes = np.sum(np.diff(np.sign(omega)) != 0)

        rows.append(dict(
            angle_target_deg=angle, trial=int(trial),
            n_samples=len(g), duration_s=float(t[-1] - t[0]) if len(t) > 1 else 0.0,
            theta0_rad=float(theta0), theta0_deg=float(np.degrees(theta0)),
            theta_final_rad=theta_final, theta_final_deg=float(np.degrees(theta_final)),
            omega_tail_std=omega_tail_std,
            min_theta_deg=float(np.degrees(theta.min())), max_theta_deg=float(np.degrees(theta.max())),
            arc_length_rad=arc_length, visc_integral=visc_integral,
            crossed_zero=crossed_zero, n_vel_sign_changes=int(sign_changes),
        ))
    return pd.DataFrame(rows).sort_values(['angle_target_deg', 'trial']).reset_index(drop=True)


# ---------------------------------------------------------------------------
# Finding 1: static-friction dead zone
# ---------------------------------------------------------------------------

def analyze_dead_zone(feat):
    """Where does the pendulum end up stuck, and at what release angle does
    it first manage to swing all the way through vertical?"""
    never_cross = feat[~feat['crossed_zero']]
    always_cross_angles = sorted(feat.loc[feat['crossed_zero'], 'angle_target_deg'].unique())
    never_cross_angles = sorted(feat.loc[~feat['crossed_zero'], 'angle_target_deg'].unique())

    threshold_lo = max(never_cross_angles) if never_cross_angles else None
    threshold_hi = min(always_cross_angles) if always_cross_angles else None

    dead_zone_half_width_deg = float(feat['theta_final_deg'].abs().max())

    return dict(
        max_release_angle_deg_with_no_crossing=threshold_lo,
        min_release_angle_deg_that_always_crosses=threshold_hi,
        dead_zone_half_width_deg_lower_bound=dead_zone_half_width_deg,
        n_trials_never_crossing=int(len(never_cross)),
        n_trials_total=int(len(feat)),
    )


# ---------------------------------------------------------------------------
# Finding 2 (primary): energy-dissipation regression
# ---------------------------------------------------------------------------

def fit_energy_dissipation(feat, k_g_reference):
    """
    Every trial starts at rest at theta0 and ends at rest at theta_final.
    Mechanical energy lost between those two rest states must equal the
    work done against friction along the path:

        k_g * [cos(theta_final) - cos(theta0)]  =  c * S  +  b * V

    where k_g = m*g*(L/2) (peak gravity torque), S = total arc length
    traveled (rad), V = integral of omega^2 dt (rad^2/s), c = Coulomb
    friction torque (Nm), b = viscous coefficient (Nm*s/rad).

    Dividing by k_g gives a regression with NO dependence on k_g at all:

        X  =  mu_c * S  +  mu_b * V,   X := cos(theta_final) - cos(theta0)

    mu_c = c/k_g (dimensionless), mu_b = b/k_g (seconds). Fit by ordinary
    least squares (with and without a free intercept, as a check that the
    intercept is ~0 as the physics predicts).
    """
    X = np.cos(feat['theta_final_rad']) - np.cos(feat['theta0_rad'])
    S = feat['arc_length_rad'].to_numpy()
    V = feat['visc_integral'].to_numpy()

    # no-intercept fit (the physically-motivated model)
    A = np.column_stack([S, V])
    coef, residuals, rank, sv = np.linalg.lstsq(A, X, rcond=None)
    mu_c, mu_b = coef
    X_pred = A @ coef
    ss_res = float(np.sum((X - X_pred) ** 2))
    ss_tot = float(np.sum((X - X.mean()) ** 2))
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    # bootstrap CI (resample trials with replacement)
    rng = np.random.default_rng(0)
    n = len(X)
    boots = []
    for _ in range(2000):
        idx = rng.integers(0, n, n)
        Ab, Xb = A[idx], X.to_numpy()[idx]
        try:
            c_b, _, _, _ = np.linalg.lstsq(Ab, Xb, rcond=None)
            boots.append(c_b)
        except np.linalg.LinAlgError:
            continue
    boots = np.array(boots)
    mu_c_ci = np.percentile(boots[:, 0], [2.5, 97.5]).tolist()
    mu_b_ci = np.percentile(boots[:, 1], [2.5, 97.5]).tolist()

    # with-intercept fit, as a physical sanity check
    A2 = np.column_stack([np.ones(n), S, V])
    coef2, *_ = np.linalg.lstsq(A2, X, rcond=None)

    result = dict(
        mu_c=float(mu_c), mu_b=float(mu_b), r2=float(r2),
        mu_c_95ci=mu_c_ci, mu_b_95ci=mu_b_ci,
        intercept_check=float(coef2[0]),  # should be close to 0
        n_trials_used=int(n),
        k_g_reference_Nm=k_g_reference,
        c_Nm_using_reference=float(mu_c * k_g_reference),
        b_Nm_s_per_rad_using_reference=float(mu_b * k_g_reference),
    )
    return result, X.to_numpy(), X_pred


# ---------------------------------------------------------------------------
# Finding 3 (cross-check): nonlinear ODE trajectory fit on swinging trials
# ---------------------------------------------------------------------------

def _ode_rhs(t, y, omega0_sq, mu_b_over_I_dummy, zeta_v, a_c, eps):
    theta, omega = y
    friction = a_c * np.tanh(omega / eps)  # smoothed Coulomb term
    domega = -omega0_sq * np.sin(theta) - zeta_v * omega - friction
    return [omega, domega]


def _simulate(params, t_eval, theta0, omega0_ic, method='RK23', rtol=1e-4, atol=1e-6):
    omega0_sq, zeta_v, a_c = params
    sol = integrate.solve_ivp(
        _ode_rhs, (t_eval[0], t_eval[-1]), [theta0, omega0_ic],
        t_eval=t_eval, args=(omega0_sq, None, zeta_v, a_c, 0.02),
        method=method, rtol=rtol, atol=atol,
    )
    if not sol.success or sol.y.shape[1] != len(t_eval):
        return None
    return sol.y[0], sol.y[1]


def fit_ode_trajectories(df, feat, min_angle_for_swing=33.0, trials_per_angle=1,
                          downsample_stride=3, max_nfev=150):
    """Fit theta''=-omega0^2 sin(theta) - zeta_v*theta' - a_c*tanh(theta'/eps)
    to a sample of trials that actually swing through vertical, by matching
    the simulated trajectory to the measured one (least squares on
    position). Only meaningful for trials with enough energy to swing --
    see the dead zone analysis for why smaller releases are excluded here.

    This is a cross-check on the (fast, closed-form) energy-dissipation fit,
    not the primary method -- each per-trial fit requires a numerical ODE
    solve inside a nonlinear least-squares loop, which is orders of
    magnitude slower than the energy method's plain linear algebra. Default
    settings (1 trial/angle, downsampled 3x, loose-but-adequate ODE
    tolerances) keep a ~20-angle sweep to well under a minute; increase
    trials_per_angle for a more thorough (slower) cross-check.
    """
    swinging = feat[feat['angle_target_deg'] >= min_angle_for_swing].copy()
    swinging = (swinging.sort_values(['angle_target_deg', 'trial'])
                .groupby('angle_target_deg', group_keys=False).head(trials_per_angle))

    fits = []
    for _, row in swinging.iterrows():
        g = df[(df.angle_target_deg == row['angle_target_deg']) & (df.trial == row['trial'])]
        g = g.sort_values('t_release_s')
        t = g['t_release_s'].to_numpy()[::downsample_stride]
        theta = g['pos_rad'].to_numpy()[::downsample_stride]
        omega = g['vel_rad_s'].to_numpy()[::downsample_stride]
        if len(t) < 15:
            continue

        def residuals(params):
            sim = _simulate(params, t, theta[0], omega[0])
            if sim is None:
                return np.full_like(theta, 1e3)
            theta_sim, _ = sim
            return theta_sim - theta

        x0 = [15.0, 0.5, 2.0]  # initial guesses: omega0^2, zeta_v, a_c
        bounds = ([0.1, 0.0, 0.0], [200.0, 20.0, 50.0])
        try:
            res = optimize.least_squares(residuals, x0, bounds=bounds, max_nfev=max_nfev)
        except Exception:
            continue
        if not res.success:
            continue
        omega0_sq, zeta_v, a_c = res.x
        rmse_deg = float(np.degrees(np.sqrt(np.mean(res.fun ** 2))))
        fits.append(dict(
            angle_target_deg=row['angle_target_deg'], trial=row['trial'],
            omega0_rad_s=float(np.sqrt(max(omega0_sq, 0))), omega0_sq=float(omega0_sq),
            zeta_v_per_s=float(zeta_v), a_c_rad_s2=float(a_c),
            rmse_deg=rmse_deg,
        ))
    return pd.DataFrame(fits)


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def make_plots(df, feat, dead_zone, energy_fit_result, X_meas, X_pred, ode_fits, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    cmap = plt.get_cmap('viridis')
    angles = sorted(df['angle_target_deg'].unique())
    norm = matplotlib.colors.Normalize(vmin=min(angles), vmax=max(angles))

    # (a) trajectory overlay, trial 1 of every 3rd angle, colored by release angle
    fig, ax = plt.subplots(figsize=(9, 5.5))
    for angle in angles[::3]:
        g = df[(df.angle_target_deg == angle) & (df.trial == 1)].sort_values('t_release_s')
        ax.plot(g['t_release_s'], g['pos_deg'], color=cmap(norm(angle)), lw=1.4)
    ax.axhline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.set_xlabel('time since release (s)')
    ax.set_ylabel('position (deg)')
    ax.set_title('Free-release trajectories (trial 1, every 3rd release angle)')
    sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label('release angle (deg)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '01_trajectories.png'), dpi=150)
    plt.close(fig)

    # (b) final rest angle vs release angle -- the dead-zone signature
    fig, ax = plt.subplots(figsize=(8, 5))
    colors = feat['crossed_zero'].map({True: cmap(0.85), False: cmap(0.15)})
    ax.scatter(feat['angle_target_deg'], feat['theta_final_deg'], c=colors, s=28, alpha=0.85,
               edgecolors='white', linewidths=0.4)
    ax.axhline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.axhspan(-dead_zone['dead_zone_half_width_deg_lower_bound'],
               dead_zone['dead_zone_half_width_deg_lower_bound'],
               color=cmap(0.15), alpha=0.08, zorder=0,
               label=f"observed dead-zone (±{dead_zone['dead_zone_half_width_deg_lower_bound']:.1f}°)")
    ax.set_xlabel('release (target) angle (deg)')
    ax.set_ylabel('final rest angle (deg)')
    ax.set_title('Where each trial comes to rest -- dark = never swung through vertical')
    ax.legend(loc='upper left', frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '02_rest_angle_vs_release.png'), dpi=150)
    plt.close(fig)

    # (c) energy regression parity plot
    fig, ax = plt.subplots(figsize=(6.5, 6))
    c = [cmap(norm(a)) for a in feat['angle_target_deg']]
    ax.scatter(X_meas, X_pred, c=c, s=26, alpha=0.85, edgecolors='white', linewidths=0.4)
    lims = [min(X_meas.min(), X_pred.min()), max(X_meas.max(), X_pred.max())]
    ax.plot(lims, lims, color='#888888', lw=1, ls='--', zorder=0, label='y = x')
    ax.set_xlabel('measured  cos(θ_final) − cos(θ₀)')
    ax.set_ylabel('predicted  μ_c·S + μ_b·V')
    ax.set_title(f"Energy-dissipation regression fit  (R² = {energy_fit_result['r2']:.4f})")
    ax.legend(loc='upper left', frameon=False)
    sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=norm)
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label('release angle (deg)')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '03_energy_fit_parity.png'), dpi=150)
    plt.close(fig)

    # (d) phase portrait for a few large-angle swinging trials
    fig, ax = plt.subplots(figsize=(7, 6))
    for angle in [45.0, 60.0, 75.0, 90.0]:
        if angle not in angles:
            continue
        g = df[(df.angle_target_deg == angle) & (df.trial == 1)].sort_values('t_release_s')
        ax.plot(g['pos_deg'], g['vel_rad_s'], color=cmap(norm(angle)), lw=1.2, label=f'{angle:g}°')
    ax.axvline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.axhline(0, color='#888888', lw=1, ls='--', zorder=0)
    ax.set_xlabel('position (deg)')
    ax.set_ylabel('velocity (rad/s)')
    ax.set_title('Phase portraits -- spiral settles off-origin (dead zone), not at 0°')
    ax.legend(frameon=False, title='release angle')
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, '04_phase_portraits.png'), dpi=150)
    plt.close(fig)

    # (e) ODE trajectory fit examples
    if len(ode_fits) > 0:
        fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
        example_angles = [a for a in [36.0, 54.0, 72.0, 90.0] if a in ode_fits['angle_target_deg'].values]
        for ax, angle in zip(axes.flat, example_angles):
            row = ode_fits[(ode_fits.angle_target_deg == angle)].iloc[0]
            g = df[(df.angle_target_deg == angle) & (df.trial == row['trial'])].sort_values('t_release_s')
            t = g['t_release_s'].to_numpy()
            theta = g['pos_rad'].to_numpy()
            omega = g['vel_rad_s'].to_numpy()
            sim = _simulate([row['omega0_sq'], row['zeta_v_per_s'], row['a_c_rad_s2']], t, theta[0], omega[0])
            ax.plot(t, np.degrees(theta), color='#333333', lw=1.6, label='measured')
            if sim is not None:
                ax.plot(t, np.degrees(sim[0]), color=cmap(0.6), lw=1.6, ls='--', label='ODE fit')
            ax.set_title(f'{angle:g}° release, trial {int(row["trial"])}  (rmse={row["rmse_deg"]:.2f}°)')
            ax.set_xlabel('t (s)')
            ax.set_ylabel('deg')
            ax.legend(frameon=False, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, '05_ode_fit_examples.png'), dpi=150)
        plt.close(fig)

        # (f) ODE-fit parameter stability vs release angle
        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        for ax, col, label in zip(
            axes, ['omega0_rad_s', 'zeta_v_per_s', 'a_c_rad_s2'],
            ['natural frequency ω₀ (rad/s)', 'viscous rate ζ (1/s)', 'Coulomb decel. a_c (rad/s²)'],
        ):
            ax.scatter(ode_fits['angle_target_deg'], ode_fits[col], color=cmap(0.4), s=26,
                       edgecolors='white', linewidths=0.4)
            ax.set_xlabel('release angle (deg)')
            ax.set_ylabel(label)
        fig.suptitle('ODE-fit parameters vs release angle (should be roughly flat if the model fits well)')
        fig.tight_layout()
        fig.savefig(os.path.join(out_dir, '06_ode_param_stability.png'), dpi=150)
        plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="Analyze pendulum_release_experiment.py output for dry friction + damping.")
    ap.add_argument('--csv', default='data/pendulum_release_sysid.csv')
    ap.add_argument('--meta', default='data/pendulum_release_sysid_meta.json')
    ap.add_argument('--out-dir', default='out')
    ap.add_argument('--swing-threshold-deg', type=float, default=None,
                     help="release angle above which trials are used for the ODE cross-check fit "
                          "(default: auto-detected from the data as the smallest angle where "
                          "all trials swing through vertical)")
    ap.add_argument('--ode-trials-per-angle', type=int, default=1,
                     help="how many trials per angle to include in the (slow) ODE trajectory-fit "
                          "cross-check; the primary energy-dissipation fit always uses all trials "
                          "regardless of this setting (default: 1)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    df, meta = load(args.csv, args.meta)
    feat = trial_features(df)
    feat.to_csv(os.path.join(args.out_dir, 'trial_features.csv'), index=False)

    dead_zone = analyze_dead_zone(feat)
    swing_threshold = args.swing_threshold_deg or dead_zone['min_release_angle_deg_that_always_crosses'] or 30.0

    k_g_ref = meta['reference_rod_mass_kg'] * G * (meta['reference_rod_length_m'] / 2.0)
    energy_result, X_meas, X_pred = fit_energy_dissipation(feat, k_g_ref)

    ode_fits = fit_ode_trajectories(df, feat, min_angle_for_swing=swing_threshold,
                                     trials_per_angle=args.ode_trials_per_angle)
    ode_fits.to_csv(os.path.join(args.out_dir, 'ode_trajectory_fits.csv'), index=False)

    # independent inertia estimate from the ODE fit's omega0 + reference k_g
    if len(ode_fits) > 0:
        omega0_med = float(ode_fits['omega0_rad_s'].median())
        I_estimate = k_g_ref / (omega0_med ** 2) if omega0_med > 0 else float('nan')
        zeta_v_med = float(ode_fits['zeta_v_per_s'].median())
        a_c_med = float(ode_fits['a_c_rad_s2'].median())
        # per-trial zeta_v/a_c trade-off diagnostic: a single ~1-swing trial
        # often can't separate viscous vs Coulomb dissipation well on its
        # own (both reduce swing amplitude similarly), so a strong negative
        # correlation here is a real identifiability warning, not noise --
        # the pooled energy-dissipation fit (Finding 2) is the one built to
        # avoid this, by combining many amplitudes where the two dissipation
        # mechanisms scale differently.
        zeta_ac_corr = float(ode_fits['zeta_v_per_s'].corr(ode_fits['a_c_rad_s2'])) if len(ode_fits) > 2 else float('nan')
        ode_summary = dict(
            n_trials_fit=int(len(ode_fits)),
            omega0_rad_s_median=omega0_med,
            zeta_v_per_s_median=zeta_v_med,
            a_c_rad_s2_median=a_c_med,
            I_estimate_kg_m2_using_reference_kg=I_estimate,
            b_Nm_s_per_rad_using_I_estimate=zeta_v_med * I_estimate,
            c_Nm_using_I_estimate=a_c_med * I_estimate,
            rmse_deg_median=float(ode_fits['rmse_deg'].median()),
            zeta_v_vs_a_c_correlation_across_trials=zeta_ac_corr,
            identifiability_caveat=(
                "Per-trial zeta_v and a_c are only weakly separable from a single "
                "~1-swing trajectory (see the correlation above and 06_ode_param_stability.png) "
                "-- trust the MEDIAN/omega0 here more than any individual trial's (zeta_v, a_c) split, "
                "and prefer Finding 2 (energy-dissipation fit) for the b vs c split itself."
            ),
        )
    else:
        ode_summary = {"note": "no trials fit successfully"}

    make_plots(df, feat, dead_zone, energy_result, X_meas, X_pred, ode_fits, args.out_dir)

    summary = dict(
        source_csv=os.path.basename(args.csv),
        source_meta=os.path.basename(args.meta),
        n_trials=len(feat),
        reference_rod_mass_kg=meta['reference_rod_mass_kg'],
        reference_rod_length_m=meta['reference_rod_length_m'],
        reference_k_g_Nm=k_g_ref,
        reference_params_caveat=meta.get('reference_params_note', ''),
        dead_zone_finding=dead_zone,
        energy_dissipation_fit=energy_result,
        ode_trajectory_fit_summary=ode_summary,
        swing_threshold_deg_used_for_ode_fit=swing_threshold,
    )
    with open(os.path.join(args.out_dir, 'sysid_summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    # console report
    print("=" * 78)
    print("PENDULUM DRY-FRICTION / DAMPING SYSID SUMMARY")
    print("=" * 78)
    print(f"{len(feat)} trials analyzed, release angles {feat.angle_target_deg.min():g}-"
          f"{feat.angle_target_deg.max():g} deg")
    print()
    print("FINDING 1 -- static friction dead zone")
    print(f"  Releases <= {dead_zone['max_release_angle_deg_with_no_crossing']:g} deg NEVER swing "
          f"through vertical within 5s ({dead_zone['n_trials_never_crossing']}/{dead_zone['n_trials_total']} "
          f"trials total never cross).")
    print(f"  Releases >= {dead_zone['min_release_angle_deg_that_always_crosses']:g} deg ALWAYS swing through.")
    print(f"  Observed final rest angles reach up to {dead_zone['dead_zone_half_width_deg_lower_bound']:.2f} deg "
          f"from vertical -- a lower bound on the static-friction dead-zone half-width.")
    print()
    print("FINDING 2 -- energy-dissipation regression (primary method, uses all trials)")
    print(f"  mu_c (Coulomb torque / peak gravity torque)  = {energy_result['mu_c']:.4f}  "
          f"(95% CI {energy_result['mu_c_95ci'][0]:.4f} .. {energy_result['mu_c_95ci'][1]:.4f})")
    print(f"  mu_b (viscous coeff / peak gravity torque)   = {energy_result['mu_b']:.4f} s  "
          f"(95% CI {energy_result['mu_b_95ci'][0]:.4f} .. {energy_result['mu_b_95ci'][1]:.4f})")
    print(f"  R^2 = {energy_result['r2']:.4f}   intercept check (should be ~0) = {energy_result['intercept_check']:.4f}")
    print(f"  Using the REFERENCE rod params (mass={meta['reference_rod_mass_kg']}kg, "
          f"length={meta['reference_rod_length_m']}m, k_g={k_g_ref:.4f} Nm) -- UNVERIFIED, see caveat:")
    print(f"    c (Coulomb friction torque) ~ {energy_result['c_Nm_using_reference']:.4f} Nm")
    print(f"    b (viscous coefficient)     ~ {energy_result['b_Nm_s_per_rad_using_reference']:.4f} Nm*s/rad")
    print()
    print(f"FINDING 3 -- ODE trajectory-fit cross-check ({ode_summary.get('n_trials_fit', 0)} swinging trials, "
          f">= {swing_threshold:g} deg release)")
    if 'omega0_rad_s_median' in ode_summary:
        print(f"  omega0 (natural frequency, no mass assumption) = {ode_summary['omega0_rad_s_median']:.3f} rad/s")
        print(f"  zeta_v (viscous rate)  = {ode_summary['zeta_v_per_s_median']:.3f} 1/s")
        print(f"  a_c (Coulomb decel.)   = {ode_summary['a_c_rad_s2_median']:.3f} rad/s^2")
        print(f"  median trajectory fit RMSE = {ode_summary['rmse_deg_median']:.2f} deg")
        print(f"  Implied inertia (using reference k_g) I ~ {ode_summary['I_estimate_kg_m2_using_reference_kg']:.5f} kg*m^2")
        print(f"    -> b ~ {ode_summary['b_Nm_s_per_rad_using_I_estimate']:.4f} Nm*s/rad, "
              f"c ~ {ode_summary['c_Nm_using_I_estimate']:.4f} Nm  (cross-check vs Finding 2)")
    print()
    print(f"Outputs written to: {os.path.abspath(args.out_dir)}/")
    print("  trial_features.csv, ode_trajectory_fits.csv, sysid_summary.json, 01..06_*.png")
    print("=" * 78)


if __name__ == '__main__':
    main()
```
