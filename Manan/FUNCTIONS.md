# Manan folder — function reference

Detailed docs + usage examples for every function in `at_can_bus.py`,
`encoder_feedback.py`, and `motor_control.py`. See `../docs/protocol_notes.md`
for the underlying CAN/AT-protocol writeup this is all built on, and
`test_scripts.py` for runnable proof that the low-level pieces work.

All examples assume:

```python
from at_can_bus import (
    ATCanBus, enable, stop, set_zero, set_run_mode_mit,
    control, decode_feedback, write_param_u8, write_param_f32,
    set_run_mode, set_run_mode_current, set_current,
    set_run_mode_velocity, set_velocity, set_velocity_limit_cur, set_velocity_accel,
    set_run_mode_position_pp, set_run_mode_position_csp,
    set_position_pp_profile, set_position_csp_limit_spd, set_location,
)
```

---

## 1. Constants (at_can_bus.py)

Quick reference for values used throughout — you won't usually call these
directly, but function signatures below refer to them.

| Name | Value | Meaning |
|---|---|---|
| `HOST_ID` | `0xFD` | Fixed convention for "the host" in the `data16` field of most frames |
| `P_MIN, P_MAX` | `-12.57, 12.57` | Position range, rad (used to pack/unpack the 16-bit position field) |
| `V_MIN, V_MAX` | `-33.0, 33.0` | Velocity range, rad/s |
| `KP_MIN, KP_MAX` | `0.0, 500.0` | Position gain range |
| `KD_MIN, KD_MAX` | `0.0, 5.0` | Damping gain range |
| `T_MIN, T_MAX` | `-14.0, 14.0` | Torque range, Nm |
| `MODE_CONTROL` … `MODE_PARAM_WRITE` | `1, 2, 3, 4, 6, 17, 18` | Frame "mode" values (control / feedback / enable / stop / set-zero / param-read / param-write) |
| `PARAM_RUN_MODE` | `0x7005` | Parameter index: 0=MIT, 1=position(PP), 2=velocity, 3=current, 5=position(CSP) |
| `PARAM_IQ_REF` | `0x7006` | Native Current mode: target Iq current (A) |
| `PARAM_SPD_REF` | `0x700A` | Native Velocity mode: target speed (rad/s) |
| `PARAM_LOC_REF` | `0x7016` | Native Position modes (PP & CSP): target position (rad) |
| `PARAM_LIMIT_SPD` | `0x7017` | Native Position/CSP mode: speed limit (rad/s) |
| `PARAM_LIMIT_CUR` | `0x7018` | Native Velocity/Position mode: current limit (A) |
| `PARAM_ACC_RAD` | `0x7022` | Native Velocity mode: acceleration (rad/s²) |
| `PARAM_VEL_MAX` | `0x7024` | Native Position/PP mode: max velocity (rad/s) |
| `PARAM_ACC_SET` | `0x7025` | Native Position/PP mode: acceleration (rad/s²) |
| `RUN_MODE_MIT` | `0` | The run mode `control()` expects |
| `RUN_MODE_POSITION_PP` (alias `RUN_MODE_POSITION`) | `1` | Native Position mode, profile-position variant - motor plans its own trajectory |
| `RUN_MODE_VELOCITY` | `2` | Native Velocity mode |
| `RUN_MODE_CURRENT` | `3` | Native Current mode |
| `RUN_MODE_POSITION_CSP` | `5` | Native Position mode, cyclic-synchronous-position variant - you stream `loc_ref` |
| `IQ_MIN, IQ_MAX` | `-16.0, 16.0` | Native Current mode command range, A |
| `CUR_LIMIT_MIN, CUR_LIMIT_MAX` | `0.0, 16.0` | Native Velocity/Position current-limit range, A |

---

## 2. `ATCanBus` — transport layer

```python
class ATCanBus(can.BusABC):
    def __init__(self, channel='/dev/ttyUSB0', bitrate=1000000, **kwargs)
```

**What it does:** opens the adapter's serial port at 921600 baud and
speaks the "AT" framing (`AT` + 4-byte id + 1-byte length + data + `\r\n`)
underneath python-can's `Message` API. This is the only class in the
folder — everything else is plain functions that take an `ATCanBus`
instance as their first argument.

**Parameters**

- `channel` (`str`) — serial port name. `'COM8'` on Windows, `/dev/ttyUSB0` on Linux.
- `bitrate` (`int`) — accepted for API compatibility with `can.BusABC`; the adapter's actual CAN bitrate (1 Mbps) is fixed in hardware, this argument isn't used to change it.

**Example**

```python
bus = ATCanBus('COM8')      # Windows
# bus = ATCanBus('/dev/ttyUSB0')  # Linux
...
bus.shutdown()
```

### `bus.send(msg, timeout=None)`

Takes a `can.Message` and writes the equivalent AT frame to the serial
port. You'll rarely call this directly — `control()`, `enable()`, etc.
build the `can.Message` for you.

### `bus.recv(timeout=1.0)`

Reads from the serial port until it finds a complete `AT...\r\n` frame or
`timeout` seconds elapse, and returns a `can.Message` (or `None` if
nothing valid arrived in time).

### `bus.shutdown()`

Closes the serial port. Always call this when you're done with the bus
(a `finally:` block or `with` statement), otherwise the port stays locked
by your process.

```python
bus = ATCanBus('COM8')
try:
    ...  # your control loop
finally:
    bus.shutdown()
```

---

## 3. Lifecycle & control functions (at_can_bus.py)

All of these take `bus` and `motor_id` as the first two arguments, plus
optional `host_id=HOST_ID` and a `timeout` for how long to wait for the
motor's reply.

### `enable(bus, motor_id, host_id=HOST_ID, timeout=0.1)`

Turns on the motor's control loop (mode 3). Call this once before sending
any `control()` frames — an unenabled motor won't respond to them.

**Returns:** the raw `can.Message` reply, or `None` if no reply arrived.

```python
bus = ATCanBus('COM8')
resp = enable(bus, motor_id=1)
if resp is None:
    print("No reply to enable() — check port/motor id/wiring.")
```

### `stop(bus, motor_id, host_id=HOST_ID, timeout=0.1)`

Disables the motor (mode 4). If the motor is currently moving, ramp its
gains down first (see `ramp_down` below) to avoid a sudden jerk.

```python
stop(bus, motor_id=1)
bus.shutdown()
```

### `set_zero(bus, motor_id, host_id=HOST_ID, timeout=0.1)`

Communication type 6: defines the shaft's *current* physical position as
0 rad. Call this while holding the shaft/rod at whatever you want "zero"
to mean (e.g. hanging straight down for a pendulum).

```python
input("Hold the rod straight down, then press Enter...")
set_zero(bus, motor_id=1)
```

### `set_run_mode_mit(bus, motor_id, host_id=HOST_ID, timeout=0.1)`

Resets the motor's `run_mode` parameter to 0 (MIT/operation mode). This
is a convenience wrapper around `write_param_u8`. **Always call this
before your first `control()` frame** — if the motor was left in
Velocity or Current mode from an earlier session, it will silently
ignore MIT-style control frames until this is sent.

```python
set_run_mode_mit(bus, motor_id=1)
```

### `write_param_u8(bus, motor_id, param_id, value, host_id=HOST_ID, timeout=0.1)`

Generic single-byte parameter write (mode 18). `set_run_mode_mit` is
built on this; you can also use it directly for other parameters.

**Parameters**

- `param_id` (`int`) — e.g. `PARAM_RUN_MODE` (`0x7005`), `PARAM_LIMIT_CUR` (`0x7018`).
- `value` (`int`, 0–255) — the byte value to write.

```python
from at_can_bus import write_param_u8, PARAM_LIMIT_CUR
write_param_u8(bus, motor_id=1, param_id=PARAM_LIMIT_CUR, value=10)  # cap current
```

### `write_param_f32(bus, motor_id, param_id, value, host_id=HOST_ID, timeout=0.1)`

Generic 32-bit float parameter write (mode 18, non-persistent) — bytes
0-1 are the `param_id` (u16), bytes 2-3 are zero, bytes 4-7 are `value`
packed as a little-endian float. This is what every native-mode command
below (`set_current`, `set_velocity`, `set_location`, the limit/profile
setters, …) is built on. You'll rarely call it directly.

```python
from at_can_bus import write_param_f32, PARAM_SPD_REF
write_param_f32(bus, motor_id=1, param_id=PARAM_SPD_REF, value=5.0)
```

### `set_run_mode(bus, motor_id, run_mode, host_id=HOST_ID, timeout=0.1)`

Writes the `run_mode` parameter (`0x7005`) — selects which native control
mode the motor's parameter-write-driven reference (`iq_ref` / `spd_ref` /
`loc_ref`) applies to. `set_run_mode_mit` and the `set_run_mode_current` /
`_velocity` / `_position_pp` / `_position_csp` wrappers below all just
call this with the matching `RUN_MODE_*` constant. Always (re-)`enable()`
after switching modes.

### `control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0, host_id=HOST_ID, timeout=0.05)`

The one function that actually drives the motor (mode 1). The motor's
own firmware computes:

```
torque_out = kp * (position - actual_position)
           + kd * (velocity - actual_velocity)
           + torque
```

so this single call covers three control styles depending on which gains
you leave at zero:

| Style | Set |
|---|---|
| Pure torque | `kp=0, kd=0, torque=<value>` |
| Velocity control | `kp=0, kd=<damping>, velocity=<target>` |
| Position hold | `kp=<stiffness>, kd=<damping>, position=<target>` |

**Returns:** the raw `can.Message` feedback reply (decode it with
`decode_feedback`), or `None` on timeout.

```python
# Read-only: zero torque, just get feedback
resp = control(bus, motor_id=1, torque=0.0)

# Hold position at pi radians with moderate stiffness
resp = control(bus, motor_id=1, position=3.14159, kp=15.0, kd=1.0)

# Pure torque push, with a little damping so it doesn't run away
resp = control(bus, motor_id=1, kd=0.5, torque=0.3)
```

### `decode_feedback(resp)`

Decodes a mode-2 feedback reply (the return value of `control`, `enable`,
etc.) into a `(position, velocity, torque, temperature)` tuple. Returns
`None` if `resp` is `None` or malformed — safe to call without checking
first.

```python
resp = control(bus, motor_id=1, torque=0.0)
fb = decode_feedback(resp)
if fb:
    pos, vel, tor, temp = fb
    print(f"pos={pos:+.3f} rad  vel={vel:+.3f} rad/s  torque={tor:+.3f} Nm  temp={temp:.1f}C")
```

---

## 3a. Native Current / Velocity / Location modes (at_can_bus.py)

Unlike `control()` (which always uses MIT/operation mode, `run_mode=0`,
and blends position/velocity/torque via `kp`/`kd`), these put the motor
into one of its own dedicated `run_mode`s and drive it with the matching
native reference parameter. Use `set_run_mode_mit` + `control()` for the
MIT/operation style; use these when you specifically want the motor
firmware's own current, speed, or position loop.

**Common pattern:** `stop()` (disable) → switch mode → `enable()` →
(optionally set limits/profile) → repeatedly call the `set_*` command
function.

**The `stop()` before switching modes is not optional.** The RobStride
manual's precautions section is explicit: *"Do not switch the control
mode when the joint is running. If you need to switch, send the command
to stop the operation before switching."* If you call `set_run_mode_*`
while the motor is already enabled (e.g. it was left running from a
previous mode, or `main()`'s shared preamble just enabled it in MIT
mode), the mode switch is silently ignored - the motor stays in whatever
mode it was already in, and the reference parameter you think you're
driving (`spd_ref`, `iq_ref`, `loc_ref`, ...) has no effect. This is
exactly the bug behind "velocity mode does nothing, target stays at
zero": `set_run_mode_velocity()` was being called while the motor was
still enabled from the earlier MIT-mode setup, so it never actually left
`run_mode=0`. `motor_control.py`'s `run_current_mode` /
`run_velocity_native_mode` / `run_location_mode` all call `stop()` first
for this reason - do the same in any custom script that calls these
`at_can_bus` functions directly.

### Current mode (`run_mode=3`)

```python
from at_can_bus import set_run_mode_current, enable, set_current, stop

stop(bus, motor_id=1)                # required: disable before switching mode
set_run_mode_current(bus, motor_id=1)
enable(bus, motor_id=1)
set_current(bus, motor_id=1, iq_ref=2.0)   # 2 A Iq command, clamped to [-16, 16] A
```

- `set_run_mode_current(bus, motor_id, host_id=HOST_ID, timeout=0.1)` — switches to native Current mode.
- `set_current(bus, motor_id, iq_ref, host_id=HOST_ID, timeout=0.1)` — commands Iq current directly, in Amps.

No position/velocity loop is involved — a free shaft will accelerate as
long as a nonzero current is held, same caution as `torque` mode.

### Velocity mode (`run_mode=2`)

```python
from at_can_bus import (
    set_run_mode_velocity, enable, set_velocity_limit_cur, set_velocity_accel, set_velocity, stop,
)

stop(bus, motor_id=1)                 # required: disable before switching mode
set_run_mode_velocity(bus, motor_id=1)
enable(bus, motor_id=1)
set_velocity_limit_cur(bus, motor_id=1, limit_cur=8.0)   # optional, clamped to [0, 16] A
set_velocity_accel(bus, motor_id=1, acc_rad=10.0)         # optional, rad/s^2
set_velocity(bus, motor_id=1, spd_ref=5.0)                # rad/s, clamped to [-33, 33]
```

This is the motor's own onboard speed loop — distinct from
`motor_control.run_velocity()`, which fakes velocity control by zeroing
`kp` in an MIT-mode `control()` frame.

### Location mode (`run_mode=1` PP, or `run_mode=5` CSP)

Two flavors, both commanded through `set_location`:

- **PP** (Profile Position, `run_mode=1`) — the motor plans its own
  trajectory to the target using `vel_max`/`acc_set`. Set the profile
  once; per the RobStride docs, PP does not support changing
  `vel_max`/`acc_set` mid-move.
- **CSP** (Cyclic Synchronous Position, `run_mode=5`) — no on-board
  trajectory planning; you stream `loc_ref` continuously and the motor
  tracks it, capped by `limit_spd`.

```python
from at_can_bus import (
    set_run_mode_position_pp, set_run_mode_position_csp, enable,
    set_position_pp_profile, set_position_csp_limit_spd, set_location, stop,
)

# PP: one-shot move, motor handles the trajectory
stop(bus, motor_id=1)                  # required: disable before switching mode
set_run_mode_position_pp(bus, motor_id=1)
enable(bus, motor_id=1)
set_position_pp_profile(bus, motor_id=1, vel_max=10.0, acc_set=20.0)
set_location(bus, motor_id=1, loc_ref=1.57)

# CSP: stream a moving setpoint yourself
stop(bus, motor_id=1)                  # required: disable before switching mode
set_run_mode_position_csp(bus, motor_id=1)
enable(bus, motor_id=1)
set_position_csp_limit_spd(bus, motor_id=1, limit_spd=10.0)
for target in trajectory:
    set_location(bus, motor_id=1, loc_ref=target)
```

- `set_run_mode_position_pp` / `set_run_mode_position_csp(bus, motor_id, host_id=HOST_ID, timeout=0.1)` — switch to the respective mode.
- `set_position_pp_profile(bus, motor_id, vel_max, acc_set, host_id=HOST_ID, timeout=0.1)` — PP only, set once before moving.
- `set_position_csp_limit_spd(bus, motor_id, limit_spd, host_id=HOST_ID, timeout=0.1)` — CSP only, caps tracking speed.
- `set_location(bus, motor_id, loc_ref, host_id=HOST_ID, timeout=0.1)` — target position, rad, clamped to `[-12.57, 12.57]`. Works for either flavor once the mode is set.

---

## 4. Low-level packing helpers (at_can_bus.py)

You generally don't need to call these yourself — `control()` and
`decode_feedback()` use them internally — but they're documented here
since `motor_control.py` and `test_scripts.py` both rely on them, and
you may need them for a custom parameter or frame type.

### `float_to_uint(x, x_min, x_max, bits)`

Packs a float into an unsigned integer of `bits` width, linearly scaled
across `[x_min, x_max]`. Clamps out-of-range input instead of overflowing.

```python
p_int = float_to_uint(1.57, P_MIN, P_MAX, 16)   # ~pi/2 rad -> 16-bit int
```

### `uint_to_float(x, x_min, x_max, bits)`

The inverse of `float_to_uint` — turns a packed integer back into a float.

```python
pos = uint_to_float(p_int, P_MIN, P_MAX, 16)    # back to ~1.57
```

### `make_id(mode, data16, motor_id)`

Builds the raw AT-frame CAN id: packs `[mode:5 bits][data16:16 bits][motor_id:8 bits]`
into the true 29-bit id, then left-shifts by 3 (the adapter's hardware
register quirk — see `docs/protocol_notes.md` §3). `enable`, `stop`,
`control`, etc. all call this for you.

```python
tx_id = make_id(mode=1, data16=HOST_ID, motor_id=1)
```

---

## 5. `encoder_feedback.py`

### `read_feedback_loop(bus, motor_id, duration=None, log_path=None)`

Continuously sends zero-torque `control()` frames and prints the decoded
feedback — a read-only loop for checking encoder sign/behavior or
logging motion by hand. Runs until Ctrl+C, or for `duration` seconds if
given.

**Parameters**

- `duration` (`float | None`) — stop automatically after this many seconds; `None` runs until interrupted.
- `log_path` (`str | None`) — if given, also writes a CSV (`t_s, pos_rad, pos_deg, vel_rad_s, torque_Nm, temp_C`) to this path.

```python
from at_can_bus import ATCanBus, enable, set_run_mode_mit, stop
from encoder_feedback import read_feedback_loop

bus = ATCanBus('COM8')
set_run_mode_mit(bus, motor_id=1)
enable(bus, motor_id=1)
read_feedback_loop(bus, motor_id=1, duration=10, log_path='swing_test.csv')
stop(bus, motor_id=1)
bus.shutdown()
```

Or from the command line:

```bash
python encoder_feedback.py --port COM8 --motor-id 1 --duration 10 --log swing_test.csv
```

---

## 6. `motor_control.py` — higher-level control helpers

### `ramp_down(bus, motor_id, steps=30, kd=2.0, dt=0.02)`

Sends `steps` zero-target, damping-only `control()` frames (kp=0,
kd=`kd`) before you disable the motor — brings it to a gentle stop
instead of an abrupt one. Always call this before `stop()` if the motor
was mid-motion.

```python
ramp_down(bus, motor_id=1)
stop(bus, motor_id=1)
```

### `run_position(bus, motor_id, target_rad, kp, kd, ramp_time)`

Reads the motor's current position, then smoothly (smoothstep-eased)
ramps the position target from there to `target_rad` over `ramp_time`
seconds, holding at `target_rad` afterward until Ctrl+C. Prints
target/actual position, velocity, and torque each cycle.

**Parameters**

- `target_rad` (`float`) — destination angle, radians.
- `kp`, `kd` (`float`) — stiffness/damping for the hold.
- `ramp_time` (`float`) — seconds to smoothly get there.

```python
import math
from at_can_bus import ATCanBus, enable, set_run_mode_mit, stop
from motor_control import run_position, ramp_down

bus = ATCanBus('COM8')
set_run_mode_mit(bus, motor_id=1)
enable(bus, motor_id=1)
run_position(bus, motor_id=1, target_rad=math.radians(90), kp=15.0, kd=1.0, ramp_time=3.0)
ramp_down(bus, motor_id=1)
stop(bus, motor_id=1)
bus.shutdown()
```

Or from the command line:

```bash
python motor_control.py position --target-deg 90 --kp 15 --kd 1 --ramp-time 3
```

### `run_velocity(bus, motor_id, target_vel, kd)`

Pure velocity control (`kp=0`) — holds a constant target angular
velocity, damped by `kd`, until Ctrl+C.

```python
run_velocity(bus, motor_id=1, target_vel=2.0, kd=1.0)   # 2 rad/s
```

```bash
python motor_control.py velocity --target-rad-s 2.0 --kd 1.0
```

### `run_torque(bus, motor_id, target_torque, kd_safety)`

Pure torque feedforward (`kp=0`), with `kd_safety` kept nonzero by
default so the shaft doesn't accelerate unbounded on a free joint. Set
`kd_safety=0` only if you have a hard current limit on the bench supply
and know what you're doing.

```python
run_torque(bus, motor_id=1, target_torque=0.3, kd_safety=0.5)
```

```bash
python motor_control.py torque --target-nm 0.3 --kd-safety 0.5
```

### `run_current_mode(bus, motor_id, target_iq, ramp_down_steps=30, dt=0.02)`

Native Current mode (`run_mode=3`): commands Iq current directly, in Amps,
via `at_can_bus.set_current`. No position/velocity loop — ramps the
current down to 0 (over `ramp_down_steps` cycles) on Ctrl+C instead of
cutting it abruptly.

```python
run_current_mode(bus, motor_id=1, target_iq=2.0)   # 2 A
```

```bash
python motor_control.py current-mode --target-a 2.0
```

### `run_velocity_native_mode(bus, motor_id, target_vel, limit_cur=None, accel=None, ramp_down_steps=30, dt=0.02)`

Native Velocity mode (`run_mode=2`): the motor's own onboard speed loop
(via `at_can_bus.set_velocity`), as opposed to `run_velocity()`'s
MIT-frame `kp=0` workaround. Optionally sets a current limit and/or
acceleration first. Ramps the target speed down to 0 on Ctrl+C.

```python
run_velocity_native_mode(bus, motor_id=1, target_vel=5.0, limit_cur=8.0, accel=10.0)
```

```bash
python motor_control.py velocity-mode --target-rad-s 5.0 --limit-cur 8.0 --accel 10.0
```

### `run_location_mode(bus, motor_id, target_rad, mode_type='csp', vel_max=10.0, acc_set=20.0, limit_spd=10.0)`

Native Location/Position mode — `mode_type='pp'` for profile-position
(motor-planned trajectory using `vel_max`/`acc_set`) or `mode_type='csp'`
(default) for cyclic-synchronous-position (streamed target, capped by
`limit_spd`). Holds/streams `target_rad` until Ctrl+C. No ramp-down
needed on stop — there's no runaway risk from a position reference the
way there is with torque/current/velocity.

```python
run_location_mode(bus, motor_id=1, target_rad=1.57, mode_type='pp', vel_max=10.0, acc_set=20.0)
run_location_mode(bus, motor_id=1, target_rad=1.57, mode_type='csp', limit_spd=10.0)
```

```bash
python motor_control.py location-mode --target-deg 90 --type pp --vel-max 10 --acc-set 20
python motor_control.py location-mode --target-deg 90 --type csp --limit-spd 10
```

---

## 7. Minimal end-to-end example

Putting the lifecycle functions together — enable, read feedback, hold a
position, then shut down safely:

```python
import time
from at_can_bus import (
    ATCanBus, set_run_mode_mit, enable, control, decode_feedback, stop,
)
from motor_control import ramp_down

PORT = 'COM8'
MOTOR_ID = 1

bus = ATCanBus(PORT)
try:
    time.sleep(3)                     # let the adapter/motor settle
    set_run_mode_mit(bus, MOTOR_ID)    # clear any leftover run mode
    time.sleep(0.1)
    enable(bus, MOTOR_ID)              # turn on the control loop
    time.sleep(0.1)

    # Hold at 0.5 rad for 2 seconds, printing feedback
    t0 = time.time()
    while time.time() - t0 < 2.0:
        resp = control(bus, MOTOR_ID, position=0.5, kp=10.0, kd=1.0)
        fb = decode_feedback(resp)
        if fb:
            pos, vel, tor, temp = fb
            print(f"pos={pos:+.3f} rad  vel={vel:+.3f} rad/s  tor={tor:+.3f} Nm")
        time.sleep(0.02)

    ramp_down(bus, MOTOR_ID)           # damp to a stop before disabling
finally:
    stop(bus, MOTOR_ID)
    bus.shutdown()
```

---

## 8. Utility Scripts

### `set_zero_logger.py`

An interactive script to calibrate the motor's mechanical zero position and log the event. Useful when a shaft is connected and you need to set the `0 rad` reference point before running control loops.

**Usage:**
```bash
python set_zero_logger.py --port COM8 --motor-id 1
```

The script asks you to hold the shaft at the desired zero position, sends the `set_zero` command, verifies the new position, and logs both pre- and post-calibration positions to `zero_calibration_log.json`.
The log is structured as a dictionary mapping each motor to its most recent configuration, ensuring that the old zero is automatically overwritten.

If you ever want to clear all configurations and wipe the file, run:
```bash
python set_zero_logger.py --clear-all
```

### `read_angle.py`

A script dedicated to reading the angle of the motor shaft relative to the zero reference. You can rotate the shaft by hand and view the output on the terminal.

The script will automatically default its COM port and motor ID to the **most recently zeroed motor** in `zero_calibration_log.json`, so you don't have to specify them manually if you just ran the calibration.

It also provides a reusable `get_position()` function for other scripts:
```python
from read_angle import get_position

pos = get_position(bus, motor_id=1)
if pos is not None:
    print(f"Angle is: {pos} rad")
```

**Usage:**
```bash
python read_angle.py
```
*(defaults to the latest calibrated motor, or you can override with `--port COM8 --motor-id 1`)*

---

## 9. Safety reminders

- Always run `encoder_feedback.py` (or the `swing` stage of
  `../src/inverted_pendulum.py`) on new hardware/wiring before trusting
  any `control()` command — confirm sign conventions first.
- `kp=0, kd=0, torque=<fixed value>` will accelerate indefinitely on a
  free shaft. Keep some `kd` damping unless you have a specific,
  deliberate reason not to (see `run_torque`'s `kd_safety`).
- Native Current mode (`current-mode` / `run_current_mode`) has the same
  runaway risk as `torque` mode - there's no damping term at all, so only
  command current you're prepared to see turn into unbounded speed on a
  free shaft. Native Velocity mode is safer (the motor's speed loop caps
  it) but still ramps down on exit rather than cutting the target
  abruptly - don't skip that step if scripting your own loop.
- Make sure the rod/shaft has full clearance before running any
  position/velocity/torque control.
- Start with a lower bench-supply current limit while testing new code;
  raise it once you trust the behavior.
