# RobStride RS00 — Rotary Inverted Pendulum

A direct-drive rotary inverted pendulum built on a RobStride RS00 quasi-direct-drive
actuator, controlled over CAN via a RobStride USB-to-CAN adapter.

The motor's own shaft *is* the pendulum pivot — no separate encoder or cart
is needed. The rod is bolted directly to the shaft; the motor's built-in
position/velocity/torque sensing and its private CAN protocol are used
directly for control.

## Hardware

- **Actuator**: RobStride RS00 (14 Nm peak, quasi-direct-drive, 14-bit
  absolute encoder)
- **Adapter**: RobStride USB-to-CAN module (CH340 + GD32, AT-protocol
  serial wrapper over CAN, 1 Mbps, 921600 baud USB link)
- **Pendulum arm**: 20×20mm aluminum extrusion, 50cm, bolted to the motor
  shaft via a mounting bracket
- **Power**: bench supply, tested at 24V / current-limited to a few amps
  for safety during development

See [`docs/protocol_notes.md`](docs/protocol_notes.md) for a full writeup
of the CAN protocol, the AT serial wrapper, and how the two connect.

## Repo layout

```
src/
  at_can_bus.py         - low-level driver: AT-protocol serial <-> python-can Message
  inverted_pendulum.py  - staged test script: zero calibration, free-swing sysID,
                          trajectory swing-up + firmware position-hold balance
  soft_balance.py       - software feedback balance controller (gravity-compensated
                          soft spring-damper, not a rigid position hold)
docs/
  protocol_notes.md     - CAN protocol reference: framing, message types,
                          parameters, and the critical bit-shift gotcha
```

## Setup

```bash
pip install pyserial python-can
```

Edit `PORT` at the top of any script to your adapter's serial port
(`/dev/ttyUSB0` on Linux, `COMx` on Windows), and `MOTOR_ID` to your
motor's actual CAN ID (check via RobStride's Motor Studio tool).

## Usage

### 1. Zero calibration

In `inverted_pendulum.py`, set `STAGE = 'zero'`, run it, and hang the rod
straight down (the stable equilibrium) when prompted. This defines that
position as `0` radians.

```bash
python src/inverted_pendulum.py
```

### 2. Free-swing check (sanity check, no control)

Set `STAGE = 'swing'`. Sends zero torque, just reads position/velocity
while you nudge the rod by hand — confirms encoder direction and gives a
feel for natural frequency/damping before trusting any control commands.

### 3. Swing-up + firmware position hold

Set `STAGE = 'balance'`. Smoothly ramps the position target from `0` to
`π` (inverted) over a few seconds, then holds there using the motor's
built-in `kp`/`kd` spring-damper. This is a **stiff** hold — good for a
first "does it work at all" check.

### 4. Soft feedback balance (the interesting one)

```bash
python src/soft_balance.py
```

Swings up the same way, then hands off to a **software-computed** control
law instead of the firmware's stiff spring:

```
τ = gravity_compensation(θ) − K·(θ − π) − D·θ̇ − ∫Ki·(θ − π)dt
```

- `gravity_compensation(θ) = m·g·(L/2)·sin(θ)` cancels the pendulum's own
  weight at every angle, so the rod doesn't need restoring force just to
  resist gravity.
- `K` (proportional) is a soft virtual spring toward vertical — much
  gentler than a stiff position-hold `kp`, since gravity is already
  cancelled separately.
- `D` (derivative) damps oscillation.
- `Ki` (integral) removes steady-state offset from an imperfect gravity
  model or bearing/cable friction — without making the response feel
  stiffer.

Try gently pushing the rod once it's balancing — it should drift and
ease back to vertical rather than snapping back rigidly.

**Tune `ROD_MASS_KG` / `ROD_LENGTH_M`** at the top of `soft_balance.py` to
your actual rod for accurate gravity compensation.

## Safety notes

- Always run the `swing` stage first on new hardware/wiring to confirm
  sign conventions before trusting any control commands.
- Pure open-loop torque commands (`kp=0, kd=0`, fixed torque, no feedback)
  will accelerate indefinitely on a free shaft — always include damping
  or feedback unless you have a specific, deliberate reason not to.
- Make sure the rod has full clearance to swing before running the
  swing-up/balance stages.
- Start with a lower bench-supply current limit while developing new
  control code; raise it once you trust the behavior.

## License

MIT
