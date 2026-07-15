"""
motor_control.py

Sends control commands to a RobStride motor over CAN: position hold,
velocity control, or pure torque - all through the same MIT-style
control() frame (see at_can_bus.py), just with different gains zeroed
out, as described in ../docs/protocol_notes.md:

    torque_out = kp * (position_target - position_actual)
               + kd * (velocity_target - velocity_actual)
               + torque_feedforward

Edit PORT / MOTOR_ID and the MODE settings below for your setup.
  Windows: 'COM6', 'COM8', ...
  Linux:   '/dev/ttyUSB0'

Safety (see project README "Safety notes"):
  - Always confirm free-swing/encoder feedback looks correct
    (encoder_feedback.py) before running this on new hardware/wiring.
  - Pure torque commands with kp=kd=0 will accelerate indefinitely on a
    free shaft - always keep some kd damping unless you have a specific
    reason not to.
  - Make sure the rod/shaft has full clearance before running.
  - Start with a lower bench-supply current limit while testing new code.
"""

import argparse
import math
import sys
import time

from at_can_bus import (
    ATCanBus,
    control,
    decode_feedback,
    enable,
    set_run_mode_mit,
    stop,
)

PORT = 'COM8'   # <-- CHANGE THIS
MOTOR_ID = 1    # <-- CHANGE THIS

CONTROL_HZ = 50
DT = 1.0 / CONTROL_HZ


def ramp_down(bus, motor_id, steps=30, kd=2.0, dt=0.02):
    """Damp to a stop (zero targets, kd only) before disabling - avoids a
    sudden jerk if the motor is mid-motion."""
    for _ in range(steps):
        control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=kd, torque=0.0)
        time.sleep(dt)


def run_position(bus, motor_id, target_rad, kp, kd, ramp_time):
    """Smoothly ramp position from the motor's current reading to
    target_rad over ramp_time seconds, then hold there until Ctrl+C."""
    resp = control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0)
    fb = decode_feedback(resp)
    start_pos = fb[0] if fb else 0.0

    print(f"Position control: {start_pos:+.3f} -> {target_rad:+.3f} rad over {ramp_time}s, "
          f"kp={kp} kd={kd}. Ctrl+C to stop.\n")
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            if t < ramp_time:
                frac = t / ramp_time
                smooth = frac * frac * (3 - 2 * frac)  # smoothstep
                target = start_pos + smooth * (target_rad - start_pos)
            else:
                target = target_rad
            resp = control(bus, motor_id, position=target, velocity=0.0, kp=kp, kd=kd, torque=0.0)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"t={t:5.2f}s target={math.degrees(target):+7.1f}deg  "
                      f"pos={math.degrees(pos):+7.1f}deg  vel={vel:+.3f}rad/s  tor={tor:+.3f}Nm")
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nStopping...")


def run_velocity(bus, motor_id, target_vel, kd):
    """Pure velocity control: kp=0, so only the velocity error drives torque."""
    print(f"Velocity control: target={target_vel:+.3f} rad/s, kd={kd}. Ctrl+C to stop.\n")
    try:
        while True:
            resp = control(bus, motor_id, position=0.0, velocity=target_vel, kp=0.0, kd=kd, torque=0.0)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"pos={pos:+.3f}rad  vel={vel:+.3f}rad/s (target {target_vel:+.3f})  tor={tor:+.3f}Nm")
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nStopping...")


def run_torque(bus, motor_id, target_torque, kd_safety):
    """Pure torque control: kp=0. kd_safety adds a small damping term so the
    shaft doesn't accelerate unbounded - set to 0 only if you know what
    you're doing and have a hard current limit on the bench supply."""
    print(f"Torque control: {target_torque:+.3f} Nm feedforward, safety kd={kd_safety}. "
          f"Ctrl+C to stop.\n")
    try:
        while True:
            resp = control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=kd_safety,
                            torque=target_torque)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"pos={pos:+.3f}rad  vel={vel:+.3f}rad/s  tor={tor:+.3f}Nm (cmd {target_torque:+.3f})")
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nStopping...")


def main():
    parser = argparse.ArgumentParser(description="Send control commands to a RobStride motor.")
    parser.add_argument('--port', default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument('--motor-id', type=int, default=MOTOR_ID, help=f"CAN motor id (default: {MOTOR_ID})")
    sub = parser.add_subparsers(dest='mode', required=True)

    p_pos = sub.add_parser('position', help="Position hold / trajectory")
    p_pos.add_argument('--target-deg', type=float, default=0.0, help="Target position in degrees")
    p_pos.add_argument('--kp', type=float, default=15.0)
    p_pos.add_argument('--kd', type=float, default=1.0)
    p_pos.add_argument('--ramp-time', type=float, default=3.0, help="Seconds to ramp to target")

    p_vel = sub.add_parser('velocity', help="Velocity control")
    p_vel.add_argument('--target-rad-s', type=float, default=0.0)
    p_vel.add_argument('--kd', type=float, default=1.0)

    p_tor = sub.add_parser('torque', help="Pure torque control (with safety damping)")
    p_tor.add_argument('--target-nm', type=float, default=0.0)
    p_tor.add_argument('--kd-safety', type=float, default=0.5,
                        help="Damping kept on even in torque mode, to avoid runaway (default 0.5)")

    args = parser.parse_args()

    print(f"Opening {args.port}, motor id {args.motor_id}...")
    bus = ATCanBus(args.port)
    try:
        print("Waiting for the adapter/motor to settle (power-cycle the motor now if it "
              "was already on)...")
        time.sleep(3)

        set_run_mode_mit(bus, args.motor_id)
        time.sleep(0.1)
        enable(bus, args.motor_id)
        time.sleep(0.1)

        if args.mode == 'position':
            run_position(bus, args.motor_id, math.radians(args.target_deg), args.kp, args.kd, args.ramp_time)
        elif args.mode == 'velocity':
            run_velocity(bus, args.motor_id, args.target_rad_s, args.kd)
        elif args.mode == 'torque':
            run_torque(bus, args.motor_id, args.target_nm, args.kd_safety)

        print("Ramping down...")
        ramp_down(bus, args.motor_id)
    finally:
        stop(bus, args.motor_id)
        bus.shutdown()
        print("Done. Safe to power off.")


if __name__ == "__main__":
    main()
