"""
pendulum_release_experiment.py

Automated free-release sweep for offline system identification (dry
friction + viscous damping) of the RS00 rotary pendulum.

For each target angle in a sweep (default 3deg..90deg in 3deg steps,
measured from the calibrated zero = hanging straight down) and for
--trials-per-angle repeats (default 5):

  1. Return to the start position (0 rad / hanging straight down, by
     default -- see --start-pos-deg) under position hold and wait for it
     to actually settle there. Every trial begins from this same known,
     at-rest state -- not just wherever the previous trial's brake
     happened to leave the shaft -- so trials are comparable and the
     approach-to-target trajectory is consistent run to run.
  2. Ramp smoothly (position hold, smoothstep trajectory) from that
     settled start position to the target angle.
  3. Hold there under position control until the shaft has actually
     settled (measured velocity below a threshold for several
     consecutive samples), not just a fixed blind delay -- this matters
     for sysID, since a trial that "releases" while still moving isn't
     a clean free-decay sample.
  4. Set ALL control outputs to zero (kp=0, kd=0, velocity=0, torque=0)
     -- a true free release, no firmware spring/damper, no feedforward.
  5. From that exact instant, poll and log (timestamp, position,
     velocity, torque, temp) as fast as the request/response protocol
     allows, for exactly 5 seconds (configurable), then stop logging.
  6. Brake (damping-only frames, like motor_control.py's ramp_down) to
     arrest any residual motion before the next trial's return-to-start
     step, so a still-swinging shaft never gets hit with a stiff kp
     command.

Every logged row is flushed to disk immediately, so a crash or Ctrl+C
partway through the sweep does not lose the trials already completed.
Ctrl+C at any point triggers the same brake+stop+shutdown sequence as a
normal exit.

IMPORTANT -- run this on the machine with the RobStride adapter actually
plugged in. It imports at_can_bus.py from this same folder.

Usage:
    python pendulum_release_experiment.py --port COM8 --motor-id 127

    # Fast sanity check before committing to the full ~150-trial sweep:
    python pendulum_release_experiment.py --quick-test

Safety (same rules as the rest of this project -- see ../README.md):
  - Run encoder_feedback.py / the swing stage first on new wiring before
    ever running this.
  - The pendulum WILL swing significantly during every trial (up to the
    target angle and back past vertical on release). Clear at least
    +-120 degrees around vertical, ideally full clearance, before
    starting -- this is an unattended 20+ minute automated sweep with no
    per-trial human confirmation once it starts.
  - Start with a lower bench-supply current limit while validating this
    script; raise it once you trust the behavior.
"""

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime

from at_can_bus import (
    ATCanBus,
    control,
    decode_feedback,
    enable,
    set_run_mode_mit,
    stop,
)

# --- defaults, matching this project's existing hardware config ---
PORT = 'COM8'
MOTOR_ID = 127          # confirmed from zero_calibration_log.json -- NOT the "1" in the docs
POLL_TIMEOUT_S = 0.02    # control()/recv() timeout per poll during logging

# --- reference physical parameters, carried over from src/soft_balance.py ---
# These are NOT re-measured by this script -- they're recorded in the run's
# metadata purely as a "last known value" breadcrumb. Verify/replace with
# your actual rod before trusting any fit that uses them.
REFERENCE_ROD_MASS_KG = 0.18
REFERENCE_ROD_LENGTH_M = 0.5


def confirm(prompt):
    return input(f"{prompt} [y/N]: ").strip().lower() == 'y'


def parse_angle_list(start_deg, stop_deg, step_deg, explicit):
    if explicit:
        return [float(x) for x in explicit.split(',')]
    angles = []
    a = start_deg
    # inclusive of stop_deg (handles float step accumulation safely)
    n_steps = round((stop_deg - start_deg) / step_deg)
    for i in range(n_steps + 1):
        angles.append(round(start_deg + i * step_deg, 6))
    return angles


def pre_flight_check(bus, motor_id):
    """Confirm the motor is actually replying before doing anything else."""
    resp = control(bus, motor_id, torque=0.0)
    fb = decode_feedback(resp)
    if fb is None:
        print("ERROR: no feedback from the motor. Check port/motor-id/wiring "
              "(see README 'Safety notes' -- run encoder_feedback.py first).")
        return None
    pos, vel, tor, temp = fb
    print(f"Pre-flight OK -- pos={math.degrees(pos):+.2f}deg  vel={vel:+.3f}rad/s  temp={temp:.1f}C")
    return fb


def move_to_angle(bus, motor_id, target_rad, kp, kd, ramp_time, poll_dt,
                   settle_vel_thresh, settle_consec, max_settle_s):
    """Smoothstep-ramp from the current actual position to target_rad, hold
    under position control, and wait until velocity has genuinely settled
    (not just a fixed delay). Returns (settled: bool, pos_at_release, vel_at_release)."""
    resp = control(bus, motor_id, torque=0.0)
    fb = decode_feedback(resp)
    start_pos = fb[0] if fb else 0.0

    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t < ramp_time:
            frac = t / ramp_time
            smooth = frac * frac * (3 - 2 * frac)  # smoothstep
            target = start_pos + smooth * (target_rad - start_pos)
        else:
            target = target_rad
        resp = control(bus, motor_id, position=target, velocity=0.0, kp=kp, kd=kd, torque=0.0)
        fb = decode_feedback(resp)
        if t >= ramp_time:
            break
        time.sleep(poll_dt)

    # Now hold at target_rad and wait for genuine settle (velocity below
    # threshold for several consecutive samples), up to max_settle_s.
    settle_t0 = time.monotonic()
    consec = 0
    last_fb = fb
    while time.monotonic() - settle_t0 < max_settle_s:
        resp = control(bus, motor_id, position=target_rad, velocity=0.0, kp=kp, kd=kd, torque=0.0)
        fb = decode_feedback(resp)
        if fb:
            last_fb = fb
            pos, vel, tor, temp = fb
            if abs(vel) < settle_vel_thresh:
                consec += 1
                if consec >= settle_consec:
                    return True, pos, vel
            else:
                consec = 0
        time.sleep(poll_dt)

    pos, vel = (last_fb[0], last_fb[1]) if last_fb else (target_rad, 0.0)
    return False, pos, vel


def release_and_log(bus, motor_id, duration_s, writer, csv_file, angle_deg, trial):
    """Zero every control output (true free release) and log feedback as
    fast as the transport allows for exactly duration_s, timed with a
    monotonic clock. Returns the number of samples logged."""
    n = 0
    t0 = time.monotonic()
    while True:
        t_release = time.monotonic() - t0
        if t_release >= duration_s:
            break
        wall_ts = time.time()
        resp = control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0,
                        timeout=POLL_TIMEOUT_S)
        fb = decode_feedback(resp)
        if fb:
            pos, vel, tor, temp = fb
            writer.writerow([
                f"{angle_deg:.3f}", trial, f"{t_release:.4f}", f"{wall_ts:.6f}",
                f"{pos:.6f}", f"{math.degrees(pos):.3f}", f"{vel:.6f}", f"{tor:.5f}", f"{temp:.1f}",
            ])
            n += 1
            csv_file.flush()
    return n


def brake(bus, motor_id, kd, duration_s, poll_dt):
    """Damping-only frames (kp=0, targets=0) to arrest residual motion
    before the next trial's position ramp -- same idea as motor_control.py's
    ramp_down(), just time-based instead of step-based."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration_s:
        control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=kd, torque=0.0)
        time.sleep(poll_dt)


def main():
    parser = argparse.ArgumentParser(
        description="Automated free-release angle sweep for pendulum system identification.")
    parser.add_argument('--port', default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument('--motor-id', type=int, default=MOTOR_ID, help=f"CAN motor id (default: {MOTOR_ID})")

    parser.add_argument('--start-deg', type=float, default=3.0)
    parser.add_argument('--stop-deg', type=float, default=90.0)
    parser.add_argument('--step-deg', type=float, default=3.0)
    parser.add_argument('--angles', default=None,
                         help="Comma-separated explicit angle list (deg), overrides start/stop/step")
    parser.add_argument('--trials-per-angle', type=int, default=5)
    parser.add_argument('--start-pos-deg', type=float, default=0.0,
                         help="position (deg from calibrated zero) every trial explicitly returns to "
                              "and settles at before ramping to the target angle (default: 0.0, i.e. "
                              "hanging straight down)")

    parser.add_argument('--release-duration-s', type=float, default=5.0,
                         help="Exact logging window after release (default: 5.0)")
    parser.add_argument('--ramp-time-s', type=float, default=2.5,
                         help="Seconds to smoothstep-ramp to each target angle")
    parser.add_argument('--hold-kp', type=float, default=15.0, help="kp used while moving/holding pre-release")
    parser.add_argument('--hold-kd', type=float, default=1.5, help="kd used while moving/holding pre-release")
    parser.add_argument('--settle-vel-thresh', type=float, default=0.05,
                         help="rad/s -- considered 'settled' below this")
    parser.add_argument('--settle-consec', type=int, default=5,
                         help="consecutive under-threshold samples required to call it settled")
    parser.add_argument('--max-settle-s', type=float, default=3.0,
                         help="give up waiting for settle after this long and release anyway (flagged in the log)")
    parser.add_argument('--brake-kd', type=float, default=2.0)
    parser.add_argument('--brake-duration-s', type=float, default=1.2)
    parser.add_argument('--poll-dt', type=float, default=0.02, help="sleep between polls outside the release window")

    parser.add_argument('--out-dir', default='.', help="directory to write the CSV + metadata JSON into")
    parser.add_argument('--out', default=None, help="explicit CSV filename (default: timestamped)")

    parser.add_argument('--quick-test', action='store_true',
                         help="Override to a tiny 2-angle, 1-trial, 1s-release run to validate the "
                              "pipeline before committing to the full sweep")
    parser.add_argument('-y', '--yes', action='store_true', help="skip the confirmation prompt")

    args = parser.parse_args()

    if args.quick_test:
        args.start_deg, args.stop_deg, args.step_deg = 15.0, 30.0, 15.0
        args.angles = None
        args.trials_per_angle = 1
        args.release_duration_s = 1.0

    angles = parse_angle_list(args.start_deg, args.stop_deg, args.step_deg, args.angles)
    start_pos_rad = math.radians(args.start_pos_deg)
    n_trials_total = len(angles) * args.trials_per_angle
    # two ramp+settle phases per trial now: return-to-start, then move-to-target
    est_trial_s = 2 * (args.ramp_time_s + args.max_settle_s * 0.3) + args.release_duration_s + args.brake_duration_s
    est_total_min = n_trials_total * est_trial_s / 60.0

    os.makedirs(args.out_dir, exist_ok=True)
    run_stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = args.out or os.path.join(args.out_dir, f"pendulum_release_sysid_{run_stamp}.csv")
    if args.out:
        csv_path = os.path.join(args.out_dir, args.out)
    meta_path = os.path.splitext(csv_path)[0] + "_meta.json"

    print("=" * 70)
    print("AUTOMATED PENDULUM FREE-RELEASE SWEEP")
    print("=" * 70)
    print(f"Port: {args.port}   Motor ID: {args.motor_id}")
    print(f"Angles ({len(angles)}): {angles}")
    print(f"Trials per angle: {args.trials_per_angle}  ->  {n_trials_total} trials total")
    print(f"Every trial returns to and settles at {args.start_pos_deg:g}deg before ramping to its target")
    print(f"Release window: {args.release_duration_s}s each, logged at full request/response rate")
    print(f"Estimated total run time: ~{est_total_min:.1f} minutes")
    print(f"Output CSV: {csv_path}")
    print(f"Output metadata: {meta_path}")
    print()
    print("SAFETY: this is an unattended automated sweep. Ensure the pendulum")
    print("has full clearance to swing at least +-120 degrees around vertical")
    print("(more if lightly damped -- it can overshoot past the release point")
    print("on the far side) before continuing. Start with a conservative")
    print("bench-supply current limit until you trust this script.")
    print("=" * 70)

    if not args.yes and not confirm("Clearance confirmed -- start the sweep?"):
        print("Aborted, nothing was sent to the motor.")
        sys.exit(0)

    metadata = {
        "run_started": datetime.now().isoformat(),
        "port": args.port,
        "motor_id": args.motor_id,
        "angles_deg": angles,
        "trials_per_angle": args.trials_per_angle,
        "start_pos_deg": args.start_pos_deg,
        "release_duration_s": args.release_duration_s,
        "ramp_time_s": args.ramp_time_s,
        "hold_kp": args.hold_kp,
        "hold_kd": args.hold_kd,
        "settle_vel_thresh_rad_s": args.settle_vel_thresh,
        "settle_consec_samples": args.settle_consec,
        "max_settle_s": args.max_settle_s,
        "brake_kd": args.brake_kd,
        "brake_duration_s": args.brake_duration_s,
        "reference_rod_mass_kg": REFERENCE_ROD_MASS_KG,
        "reference_rod_length_m": REFERENCE_ROD_LENGTH_M,
        "reference_params_note": ("Carried over from src/soft_balance.py as a 'last known value' "
                                   "only -- NOT re-measured by this script. Verify before use in fitting."),
        "angle_convention": "degrees from calibrated zero (hanging straight down = 0 rad)",
        "csv_columns": ["angle_target_deg", "trial", "t_release_s", "timestamp_unix",
                         "pos_rad", "pos_deg", "vel_rad_s", "torque_Nm", "temp_C"],
        "trials": [],   # filled in as each trial completes (actual release pos/vel, settle status)
    }

    print(f"\nOpening {args.port}, motor id {args.motor_id}...")
    bus = ATCanBus(args.port)
    csv_file = open(csv_path, 'w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow(["angle_target_deg", "trial", "t_release_s", "timestamp_unix",
                      "pos_rad", "pos_deg", "vel_rad_s", "torque_Nm", "temp_C"])
    csv_file.flush()

    completed = 0
    try:
        print("Waiting for the adapter/motor to settle...")
        time.sleep(3)
        set_run_mode_mit(bus, args.motor_id)
        time.sleep(0.1)
        enable(bus, args.motor_id)
        time.sleep(0.1)

        if pre_flight_check(bus, args.motor_id) is None:
            print("Aborting -- fix the connection before running the sweep.")
            return

        sweep_t0 = time.monotonic()
        for angle_deg in angles:
            target_rad = math.radians(angle_deg)
            for trial in range(1, args.trials_per_angle + 1):
                elapsed_min = (time.monotonic() - sweep_t0) / 60.0
                print(f"\n[{completed + 1}/{n_trials_total}]  angle={angle_deg:g}deg  "
                      f"trial={trial}/{args.trials_per_angle}  (elapsed {elapsed_min:.1f} min)")

                print(f"  Returning to start position ({args.start_pos_deg:g}deg) and settling...")
                start_settled, start_pos, start_vel = move_to_angle(
                    bus, args.motor_id, start_pos_rad, args.hold_kp, args.hold_kd,
                    args.ramp_time_s, args.poll_dt,
                    args.settle_vel_thresh, args.settle_consec, args.max_settle_s,
                )
                if not start_settled:
                    print(f"  WARNING: did not settle at start position within {args.max_settle_s}s "
                          f"(pos={math.degrees(start_pos):+.2f}deg vel={start_vel:+.3f}rad/s) -- "
                          f"proceeding to target ramp anyway, flagged in metadata.")

                print(f"  Moving to {angle_deg:g}deg ...")
                settled, release_pos, release_vel = move_to_angle(
                    bus, args.motor_id, target_rad, args.hold_kp, args.hold_kd,
                    args.ramp_time_s, args.poll_dt,
                    args.settle_vel_thresh, args.settle_consec, args.max_settle_s,
                )
                if not settled:
                    print(f"  WARNING: did not settle within {args.max_settle_s}s "
                          f"(vel={release_vel:+.3f} rad/s at release) -- releasing anyway, flagged in metadata.")
                print(f"  Releasing (all outputs zero) -- actual pos={math.degrees(release_pos):+.2f}deg "
                      f"vel={release_vel:+.3f}rad/s -- logging for {args.release_duration_s}s...")

                n_samples = release_and_log(bus, args.motor_id, args.release_duration_s,
                                             writer, csv_file, angle_deg, trial)
                achieved_hz = n_samples / args.release_duration_s if args.release_duration_s > 0 else 0.0
                print(f"  Logged {n_samples} samples over {args.release_duration_s}s (~{achieved_hz:.1f} Hz).")

                metadata["trials"].append({
                    "angle_target_deg": angle_deg,
                    "trial": trial,
                    "start_pos_settled": start_settled,
                    "actual_start_pos_rad": start_pos,
                    "actual_start_vel_rad_s": start_vel,
                    "settled_before_release": settled,
                    "actual_pos_at_release_rad": release_pos,
                    "actual_vel_at_release_rad_s": release_vel,
                    "n_samples_logged": n_samples,
                })

                print("  Braking before next trial...")
                brake(bus, args.motor_id, args.brake_kd, args.brake_duration_s, args.poll_dt)

                completed += 1

        print(f"\nSweep complete: {completed}/{n_trials_total} trials logged to {csv_path}")

    except KeyboardInterrupt:
        print(f"\nInterrupted after {completed}/{n_trials_total} trials -- "
              f"partial data already saved to {csv_path}.")
    finally:
        print("Ramping down / braking to a stop...")
        try:
            brake(bus, args.motor_id, kd=2.0, duration_s=0.6, poll_dt=0.02)
        except Exception:
            pass
        stop(bus, args.motor_id)
        bus.shutdown()
        csv_file.close()
        metadata["run_finished"] = datetime.now().isoformat()
        metadata["trials_completed"] = completed
        metadata["trials_planned"] = n_trials_total
        with open(meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        print(f"Metadata written to {meta_path}")
        print("Motor stopped, port closed. Safe to power off.")


if __name__ == "__main__":
    main()