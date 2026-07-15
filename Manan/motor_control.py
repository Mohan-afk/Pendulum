"""
motor_control.py

Sends control commands to a RobStride motor over CAN. Two families of
control modes are available:

  - MIT-style ('position', 'velocity', 'torque'): all three go through the
    same MIT-mode control() frame (see at_can_bus.py, run_mode=0), just
    with different gains zeroed out, as described in
    ../docs/protocol_notes.md:

        torque_out = kp * (position_target - position_actual)
                   + kd * (velocity_target - velocity_actual)
                   + torque_feedforward

  - Native ('current-mode', 'velocity-mode', 'location-mode'): each puts
    the motor into its own dedicated run_mode (Current=3, Velocity=2,
    Position PP=1/CSP=5) and drives it with the matching native reference
    parameter (iq_ref / spd_ref / loc_ref) instead of the shared MIT
    frame. Use these when you want the motor firmware's own current,
    speed, or position loop rather than the MIT kp/kd blend.

Edit PORT / MOTOR_ID and the MODE settings below for your setup.
  Windows: 'COM6', 'COM8', ...
  Linux:   '/dev/ttyUSB0'

Safety (see project README "Safety notes"):
  - Always confirm free-swing/encoder feedback looks correct
    (encoder_feedback.py) before running this on new hardware/wiring.
  - Pure torque/current commands with no damping will accelerate
    indefinitely on a free shaft - always keep some kd damping (MIT modes)
    or a sane current/speed limit (native modes) unless you have a
    specific reason not to.
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
    set_current,
    set_location,
    set_position_csp_limit_spd,
    set_position_pp_profile,
    set_run_mode_current,
    set_run_mode_mit,
    set_run_mode_position_csp,
    set_run_mode_position_pp,
    set_run_mode_velocity,
    set_velocity,
    set_velocity_accel,
    set_velocity_limit_cur,
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


def run_current_mode(bus, motor_id, target_iq, ramp_down_steps=30, dt=0.02):
    """Native Current mode (run_mode=3): commands Iq current directly, in
    Amps - no position/velocity loop involved, so the shaft will
    accelerate as long as a nonzero current is held on a free joint. Ramps
    the current down to 0 before returning (Ctrl+C to stop)."""
    # Per the RobStride manual's precautions: "Do not switch the control
    # mode when the joint is running. If you need to switch, send the
    # command to stop the operation before switching." main() enables the
    # motor in MIT mode before dispatching here, so we must disable before
    # touching run_mode or the switch is silently ignored and the motor
    # stays in MIT mode (iq_ref writes then have no effect).
    stop(bus, motor_id)
    time.sleep(0.05)
    set_run_mode_current(bus, motor_id)
    time.sleep(0.05)
    enable(bus, motor_id)
    time.sleep(0.05)

    print(f"Current mode (native): target={target_iq:+.3f} A. Ctrl+C to stop.\n")
    try:
        while True:
            resp = set_current(bus, motor_id, target_iq)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"pos={pos:+.3f}rad  vel={vel:+.3f}rad/s  tor={tor:+.3f}Nm (iq cmd {target_iq:+.3f}A)")
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nStopping - ramping current to 0...")
        for i in range(ramp_down_steps, 0, -1):
            set_current(bus, motor_id, target_iq * i / ramp_down_steps)
            time.sleep(dt)
        set_current(bus, motor_id, 0.0)


def run_velocity_native_mode(bus, motor_id, target_vel, limit_cur=None, accel=None,
                              ramp_down_steps=30, dt=0.02):
    """Native Velocity mode (run_mode=2): the motor's own firmware closes
    the speed loop (distinct from the MIT-frame kp=0 workaround in
    run_velocity() above). Optionally caps current / sets acceleration
    first. Ramps the target speed down to 0 before returning (Ctrl+C to
    stop)."""
    # See run_current_mode() above: the manual requires the motor be
    # stopped before switching run_mode, or the switch (and everything
    # depending on it, like spd_ref) is silently ignored.
    stop(bus, motor_id)
    time.sleep(0.05)
    set_run_mode_velocity(bus, motor_id)
    time.sleep(0.05)
    enable(bus, motor_id)
    time.sleep(0.05)
    if limit_cur is not None:
        set_velocity_limit_cur(bus, motor_id, limit_cur)
    if accel is not None:
        set_velocity_accel(bus, motor_id, accel)

    print(f"Velocity mode (native): target={target_vel:+.3f} rad/s. Ctrl+C to stop.\n")
    try:
        while True:
            resp = set_velocity(bus, motor_id, target_vel)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"pos={pos:+.3f}rad  vel={vel:+.3f}rad/s (target {target_vel:+.3f})  tor={tor:+.3f}Nm")
            time.sleep(DT)
    except KeyboardInterrupt:
        print("\nStopping - ramping speed to 0...")
        for i in range(ramp_down_steps, 0, -1):
            set_velocity(bus, motor_id, target_vel * i / ramp_down_steps)
            time.sleep(dt)
        set_velocity(bus, motor_id, 0.0)


def run_location_mode(bus, motor_id, target_rad, mode_type='csp',
                       vel_max=10.0, acc_set=20.0, limit_spd=10.0):
    """Native Location/Position mode - two flavors:

      'pp'  (run_mode=1): the motor plans its own trajectory to
            target_rad using vel_max/acc_set. Set the profile once (PP
            doesn't support changing it mid-move), send the target once,
            then just hold/observe.
      'csp' (run_mode=5): no on-board trajectory planning - loc_ref is
            (re-)sent every cycle here, capped by limit_spd, so this is
            also how you'd stream a moving setpoint.

    Holds/streams target_rad until Ctrl+C. Position modes don't need a
    ramp-down (there's no runaway risk from a torque/current/velocity
    reference) - stop() alone is sufficient afterward."""
    if mode_type not in ('pp', 'csp'):
        raise ValueError("mode_type must be 'pp' or 'csp'")

    # See run_current_mode() above: the manual requires the motor be
    # stopped before switching run_mode, or the switch is silently ignored.
    stop(bus, motor_id)
    time.sleep(0.05)

    if mode_type == 'pp':
        set_run_mode_position_pp(bus, motor_id)
        time.sleep(0.05)
        enable(bus, motor_id)
        time.sleep(0.05)
        set_position_pp_profile(bus, motor_id, vel_max=vel_max, acc_set=acc_set)
        print(f"Location mode (native, PP): target={target_rad:+.3f} rad, "
              f"vel_max={vel_max} rad/s, acc_set={acc_set} rad/s^2. Ctrl+C to stop.\n")
    else:
        set_run_mode_position_csp(bus, motor_id)
        time.sleep(0.05)
        enable(bus, motor_id)
        time.sleep(0.05)
        set_position_csp_limit_spd(bus, motor_id, limit_spd)
        print(f"Location mode (native, CSP): target={target_rad:+.3f} rad, "
              f"limit_spd={limit_spd} rad/s. Ctrl+C to stop.\n")

    try:
        while True:
            resp = set_location(bus, motor_id, target_rad)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"target={math.degrees(target_rad):+7.1f}deg  pos={math.degrees(pos):+7.1f}deg  "
                      f"vel={vel:+.3f}rad/s  tor={tor:+.3f}Nm")
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

    p_cur_mode = sub.add_parser('current-mode',
                                 help="Native Current mode (run_mode=3): direct Iq current command")
    p_cur_mode.add_argument('--target-a', type=float, default=0.0, help="Target Iq current, Amps (-16..16)")

    p_vel_mode = sub.add_parser('velocity-mode',
                                 help="Native Velocity mode (run_mode=2): motor's own onboard speed loop")
    p_vel_mode.add_argument('--target-rad-s', type=float, default=0.0)
    p_vel_mode.add_argument('--limit-cur', type=float, default=None,
                             help="Optional current limit, Amps (0..16)")
    p_vel_mode.add_argument('--accel', type=float, default=None,
                             help="Optional acceleration, rad/s^2")

    p_loc_mode = sub.add_parser('location-mode',
                                 help="Native Location/Position mode (run_mode=1 PP or 5 CSP)")
    p_loc_mode.add_argument('--target-deg', type=float, default=0.0)
    p_loc_mode.add_argument('--type', choices=['pp', 'csp'], default='csp',
                             help="pp = motor-planned trajectory (vel_max/acc_set); "
                                  "csp = streamed position target (limit_spd) (default: csp)")
    p_loc_mode.add_argument('--vel-max', type=float, default=10.0, help="PP mode: max velocity, rad/s")
    p_loc_mode.add_argument('--acc-set', type=float, default=20.0, help="PP mode: acceleration, rad/s^2")
    p_loc_mode.add_argument('--limit-spd', type=float, default=10.0, help="CSP mode: speed limit, rad/s")

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
            print("Ramping down...")
            ramp_down(bus, args.motor_id)
        elif args.mode == 'velocity':
            run_velocity(bus, args.motor_id, args.target_rad_s, args.kd)
            print("Ramping down...")
            ramp_down(bus, args.motor_id)
        elif args.mode == 'torque':
            run_torque(bus, args.motor_id, args.target_nm, args.kd_safety)
            print("Ramping down...")
            ramp_down(bus, args.motor_id)
        elif args.mode == 'current-mode':
            # re-selects run_mode=3 itself (disabling first, per the
            # manual); the preceding set_run_mode_mit above is just the
            # shared "start from a known state" step.
            run_current_mode(bus, args.motor_id, args.target_a)
        elif args.mode == 'velocity-mode':
            run_velocity_native_mode(bus, args.motor_id, args.target_rad_s,
                                      limit_cur=args.limit_cur, accel=args.accel)
        elif args.mode == 'location-mode':
            run_location_mode(bus, args.motor_id, math.radians(args.target_deg), mode_type=args.type,
                               vel_max=args.vel_max, acc_set=args.acc_set, limit_spd=args.limit_spd)

        # Leave the motor's run_mode back at MIT/operation (0) regardless of
        # which mode we just ran, so a subsequent script invocation - or
        # ramp_down()'s control() calls above - always starts from a known
        # state instead of silently no-op'ing against a leftover native
        # mode. Must disable first (stop()) - the manual is explicit that
        # run_mode writes are ignored while the motor is still enabled/
        # running, and the native-mode functions above return without
        # disabling themselves.
        stop(bus, args.motor_id)
        time.sleep(0.05)
        set_run_mode_mit(bus, args.motor_id)
    finally:
        stop(bus, args.motor_id)
        bus.shutdown()
        print("Done. Safe to power off.")


if __name__ == "__main__":
    main()
