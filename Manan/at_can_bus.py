"""
at_can_bus.py

Low-level driver + RobStride "AT" protocol helpers for the RobStride
USB-to-CAN adapter, used by the encoder_feedback.py and motor_control.py
scripts in this folder.

This mirrors src/at_can_bus.py from the main project (the raw serial
<-> can.Message transport), plus the shared protocol constants/helpers
that src/inverted_pendulum.py and src/soft_balance.py each duplicated
locally - collected here once so the other scripts in this folder can
just `from at_can_bus import ...` instead of re-typing them.

Background (see ../docs/protocol_notes.md for the full writeup):
  - The adapter doesn't show up as a normal SocketCAN device. It's a
    921600-baud serial port that wraps/unwraps real CAN frames inside a
    small "AT" envelope:

        41 54  [4-byte CAN id]  [1-byte DLC]  [0-8 bytes data]  0D 0A
        "AT"                                                    "\\r\\n"

  - The 4-byte "CAN id" in that envelope is NOT the logical id directly -
    it's the true 29-bit id left-shifted by 3 bits (a quirk of the
    adapter's hardware register). _make_id() below builds it correctly.
  - The true id packs as [ mode: 5 bits ][ data16: 16 bits ][ motor_id: 8 bits ].
"""

import struct
import time

import can
import serial


# ---------------------------------------------------------------------------
# Transport layer: AT-protocol serial <-> python-can Message
# ---------------------------------------------------------------------------

class ATCanBus(can.BusABC):
    """python-can BusABC implementation for the RobStride USB-to-CAN adapter."""

    def __init__(self, channel='/dev/ttyUSB0', bitrate=1000000, **kwargs):
        self.ser = serial.Serial(channel, baudrate=921600, timeout=0.005)
        self._buffer = bytearray()
        time.sleep(0.3)  # let the adapter settle right after opening the port
        super().__init__(channel, **kwargs)

    def send(self, msg, timeout=None):
        can_id = int(msg.arbitration_id)
        data = bytes(msg.data)
        frame = b'AT' + can_id.to_bytes(4, 'big') + bytes([len(data)]) + data + b'\r\n'
        self.ser.write(frame)

    def recv(self, timeout=1.0):
        end = time.time() + (timeout or 1.0)
        while time.time() < end:
            chunk = self.ser.read(64)
            if chunk:
                self._buffer += chunk
            start = self._buffer.find(b'AT')
            if start == -1:
                if self._buffer.endswith(b'A'):
                    self._buffer = self._buffer[-1:]
                else:
                    self._buffer.clear()
                continue
            end_idx = self._buffer.find(b'\r\n', start)
            if end_idx == -1:
                self._buffer = self._buffer[start:]
                continue
            frame = self._buffer[start:end_idx + 2]
            self._buffer = self._buffer[end_idx + 2:]
            if len(frame) < 9:
                continue
            can_id = int.from_bytes(frame[2:6], 'big')
            dlc = frame[6]
            if len(frame) < 7 + dlc + 2:
                continue
            data = frame[7:7 + dlc]
            return can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True,
                timestamp=time.time(),
            )
        return None

    def shutdown(self):
        self.ser.close()
        # Let can.BusABC know we actually cleaned up, so its __del__ doesn't
        # log a spurious "was not properly shut down" warning on GC even
        # though the serial port was closed correctly right above.
        super().shutdown()

    @staticmethod
    def _detect_available_configs():
        return []


# ---------------------------------------------------------------------------
# Protocol constants (RS00 defaults - override if your motor differs)
# ---------------------------------------------------------------------------

HOST_ID = 0xFD

# Fixed-point ranges used to pack/unpack the 16-bit fields in control frames.
P_MIN, P_MAX = -12.57, 12.57   # rad
V_MIN, V_MAX = -33.0, 33.0     # rad/s
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -14.0, 14.0     # Nm

# "mode" values: top 5 bits of the true (un-shifted) 29-bit CAN id.
MODE_CONTROL = 1
MODE_FEEDBACK = 2
MODE_ENABLE = 3
MODE_STOP = 4
MODE_SET_ZERO = 6
MODE_PARAM_READ = 17
MODE_PARAM_WRITE = 18

# Parameter indices (used with MODE_PARAM_WRITE / MODE_PARAM_READ).
PARAM_RUN_MODE = 0x7005    # 0=MIT/operation, 1=position(PP), 2=velocity, 3=current, 5=position(CSP)
PARAM_IQ_REF = 0x7006      # target Iq current (A), native Current mode
PARAM_SPD_REF = 0x700A     # target velocity, native Velocity mode
PARAM_LOC_REF = 0x7016     # target position (rad), native Position modes (PP & CSP)
PARAM_LIMIT_SPD = 0x7017   # speed limit (rad/s), native Position/CSP mode
PARAM_LIMIT_CUR = 0x7018   # max current
PARAM_ACC_RAD = 0x7022     # acceleration, native Velocity mode
PARAM_VEL_MAX = 0x7024     # max velocity (rad/s), native Position/PP mode
PARAM_ACC_SET = 0x7025     # acceleration (rad/s^2), native Position/PP mode

RUN_MODE_MIT = 0
RUN_MODE_POSITION = 1          # Position/PP (profile position - motor plans its own trajectory)
RUN_MODE_POSITION_PP = RUN_MODE_POSITION
RUN_MODE_VELOCITY = 2
RUN_MODE_CURRENT = 3
RUN_MODE_POSITION_CSP = 5      # Position/CSP (cyclic synchronous position - you stream loc_ref)

# Native command ranges, per the RobStride protocol docs.
IQ_MIN, IQ_MAX = -16.0, 16.0            # A, Current mode (iq_ref)
CUR_LIMIT_MIN, CUR_LIMIT_MAX = 0.0, 16.0  # A, Velocity/Position current limit (limit_cur)


def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(x, x_min, x_max, bits):
    span = x_max - x_min
    return float(x) / ((1 << bits) - 1) * span + x_min


def make_id(mode, data16, motor_id):
    """Build the raw AT-frame CAN id from the true 29-bit id, left-shifted
    3 bits (the adapter's hardware register quirk)."""
    true_id = (mode << 24) | (data16 << 8) | motor_id
    return true_id << 3


def enable(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Turn on the motor's control loop (mode 3)."""
    tx_id = make_id(MODE_ENABLE, host_id, motor_id)
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True))
    return bus.recv(timeout=timeout)


def stop(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Disable the motor (mode 4). Ramp gains down first if it's moving."""
    tx_id = make_id(MODE_STOP, host_id, motor_id)
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True))
    return bus.recv(timeout=timeout)


def set_zero(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Communication type 6: set the current shaft position as mechanical
    zero. Call this while the shaft is held at the position you want to
    define as 0 rad."""
    tx_id = make_id(MODE_SET_ZERO, host_id, motor_id)
    data = bytes([1, 0, 0, 0, 0, 0, 0, 0])
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=timeout)


def write_param_u8(bus, motor_id, param_id, value, host_id=HOST_ID, timeout=0.1):
    tx_id = make_id(MODE_PARAM_WRITE, host_id, motor_id)
    data = struct.pack('<HH', param_id, 0) + bytes([value]) + bytes(3)
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=timeout)


def write_param_f32(bus, motor_id, param_id, value, host_id=HOST_ID, timeout=0.1):
    """Generic 32-bit float parameter write (mode 18, non-persistent):
    bytes 0-1 = param index (u16), bytes 2-3 = 0x00, bytes 4-7 = value
    (f32, little-endian). This is what drives every native run-mode
    reference/limit parameter - iq_ref, spd_ref, loc_ref, limit_cur,
    limit_spd, acc_rad, vel_max, acc_set, etc."""
    tx_id = make_id(MODE_PARAM_WRITE, host_id, motor_id)
    data = struct.pack('<HHf', param_id, 0, float(value))
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=timeout)


def set_run_mode(bus, motor_id, run_mode, host_id=HOST_ID, timeout=0.1):
    """Write the run_mode parameter (0x7005), selecting which native control
    mode the motor's PARAM_WRITE-driven reference applies to (see the
    RUN_MODE_* constants). Set this (and re-`enable()`) before switching
    between native Current/Velocity/Position command styles."""
    return write_param_u8(bus, motor_id, PARAM_RUN_MODE, run_mode, host_id, timeout)


def set_run_mode_mit(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Reset to MIT/operation mode (0) - required before sending control()
    frames if the motor was previously left in a native Velocity/Current/
    Position mode."""
    return set_run_mode(bus, motor_id, RUN_MODE_MIT, host_id, timeout)


# ---------------------------------------------------------------------------
# Native Current Mode (run_mode = 3): direct Iq (torque-producing) current
# command, no position/velocity loop involved.
#
#   set_run_mode_current(bus, motor_id)
#   enable(bus, motor_id)
#   set_current(bus, motor_id, iq_ref=2.0)
# ---------------------------------------------------------------------------

def set_run_mode_current(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Switch the motor into native Current mode (run_mode=3). Call this,
    then enable(), before set_current()."""
    return set_run_mode(bus, motor_id, RUN_MODE_CURRENT, host_id, timeout)


def set_current(bus, motor_id, iq_ref, host_id=HOST_ID, timeout=0.1):
    """Command the Iq current directly, in Amps (clamped to [-16, 16] A).
    Motor must already be in Current mode (set_run_mode_current) and
    enabled."""
    iq_ref = max(IQ_MIN, min(IQ_MAX, iq_ref))
    return write_param_f32(bus, motor_id, PARAM_IQ_REF, iq_ref, host_id, timeout)


# ---------------------------------------------------------------------------
# Native Velocity Mode (run_mode = 2): closed-loop speed control done by the
# motor's own firmware (distinct from the MIT-frame kp=0 workaround in
# motor_control.run_velocity).
#
#   set_run_mode_velocity(bus, motor_id)
#   enable(bus, motor_id)
#   set_velocity_limit_cur(bus, motor_id, limit_cur=8.0)   # optional
#   set_velocity(bus, motor_id, spd_ref=5.0)
# ---------------------------------------------------------------------------

def set_run_mode_velocity(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Switch the motor into native Velocity mode (run_mode=2). Call this,
    then enable(), before set_velocity()."""
    return set_run_mode(bus, motor_id, RUN_MODE_VELOCITY, host_id, timeout)


def set_velocity_limit_cur(bus, motor_id, limit_cur, host_id=HOST_ID, timeout=0.1):
    """Cap the current (Amps, clamped to [0, 16]) Velocity mode is allowed
    to use to reach the commanded speed. Optional - set once before/while
    running set_velocity()."""
    limit_cur = max(CUR_LIMIT_MIN, min(CUR_LIMIT_MAX, limit_cur))
    return write_param_f32(bus, motor_id, PARAM_LIMIT_CUR, limit_cur, host_id, timeout)


def set_velocity_accel(bus, motor_id, acc_rad, host_id=HOST_ID, timeout=0.1):
    """Set the acceleration (rad/s^2) Velocity mode ramps to a new target
    speed with. Optional."""
    return write_param_f32(bus, motor_id, PARAM_ACC_RAD, acc_rad, host_id, timeout)


def set_velocity(bus, motor_id, spd_ref, host_id=HOST_ID, timeout=0.1):
    """Command the target speed, in rad/s (clamped to [-33, 33]). Motor
    must already be in Velocity mode (set_run_mode_velocity) and enabled."""
    spd_ref = max(V_MIN, min(V_MAX, spd_ref))
    return write_param_f32(bus, motor_id, PARAM_SPD_REF, spd_ref, host_id, timeout)


# ---------------------------------------------------------------------------
# Native Location/Position Mode - two flavors, both driven by loc_ref:
#
#   PP  (run_mode = 1): the motor plans its own trajectory to loc_ref using
#       vel_max/acc_set - set the profile once, then send loc_ref once per
#       move. Per the docs, PP does not support changing vel_max/acc_set
#       mid-move.
#   CSP (run_mode = 5): no on-board trajectory planning - stream loc_ref
#       continuously yourself and the motor tracks it, capped by
#       limit_spd.
#
#   PP example:
#     set_run_mode_position_pp(bus, motor_id)
#     enable(bus, motor_id)
#     set_position_pp_profile(bus, motor_id, vel_max=10.0, acc_set=20.0)
#     set_location(bus, motor_id, loc_ref=1.57)
#
#   CSP example:
#     set_run_mode_position_csp(bus, motor_id)
#     enable(bus, motor_id)
#     set_position_csp_limit_spd(bus, motor_id, limit_spd=10.0)
#     for target in trajectory:
#         set_location(bus, motor_id, loc_ref=target)
# ---------------------------------------------------------------------------

def set_run_mode_position_pp(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Switch the motor into native Position/PP mode (run_mode=1)."""
    return set_run_mode(bus, motor_id, RUN_MODE_POSITION_PP, host_id, timeout)


def set_run_mode_position_csp(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Switch the motor into native Position/CSP mode (run_mode=5)."""
    return set_run_mode(bus, motor_id, RUN_MODE_POSITION_CSP, host_id, timeout)


def set_position_pp_profile(bus, motor_id, vel_max, acc_set, host_id=HOST_ID, timeout=0.1):
    """Set the PP-mode trajectory profile (max velocity rad/s, acceleration
    rad/s^2). Set this once before set_location() - PP mode doesn't support
    changing it mid-move."""
    write_param_f32(bus, motor_id, PARAM_VEL_MAX, vel_max, host_id, timeout)
    return write_param_f32(bus, motor_id, PARAM_ACC_SET, acc_set, host_id, timeout)


def set_position_csp_limit_spd(bus, motor_id, limit_spd, host_id=HOST_ID, timeout=0.1):
    """Set the CSP-mode speed limit (rad/s) - caps how fast the motor is
    allowed to move while following a streamed loc_ref."""
    return write_param_f32(bus, motor_id, PARAM_LIMIT_SPD, limit_spd, host_id, timeout)


def set_location(bus, motor_id, loc_ref, host_id=HOST_ID, timeout=0.1):
    """Command the target position, in rad (clamped to [-12.57, 12.57]).
    Works for both PP and CSP - the motor must already be in one of those
    run modes (set_run_mode_position_pp / _csp) and enabled. Call this once
    per move in PP mode; stream new targets continuously in CSP mode."""
    loc_ref = max(P_MIN, min(P_MAX, loc_ref))
    return write_param_f32(bus, motor_id, PARAM_LOC_REF, loc_ref, host_id, timeout)


def control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0,
            host_id=HOST_ID, timeout=0.05):
    """
    Send one MIT-mode control frame (mode 1):

        torque_out = kp * (position - actual_position)
                   + kd * (velocity - actual_velocity)
                   + torque

    Covers pure torque control (kp=kd=0), velocity control (kp=0), and
    position hold (kp, kd both set), depending on which gains are zero.
    """
    t_int = float_to_uint(torque, T_MIN, T_MAX, 16)
    tx_id = make_id(MODE_CONTROL, t_int, motor_id)
    p_int = float_to_uint(position, P_MIN, P_MAX, 16)
    v_int = float_to_uint(velocity, V_MIN, V_MAX, 16)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16)
    data = struct.pack('>HHHH', p_int, v_int, kp_int, kd_int)
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=timeout)


def decode_feedback(resp):
    """Decode a mode-2 feedback frame into (position, velocity, torque, temp_C)."""
    if resp is None or len(resp.data) < 8:
        return None
    p16, v16, t16, temp16 = struct.unpack('>HHHH', bytes(resp.data[:8]))
    pos = uint_to_float(p16, P_MIN, P_MAX, 16)
    vel = uint_to_float(v16, V_MIN, V_MAX, 16)
    tor = uint_to_float(t16, T_MIN, T_MAX, 16)
    return pos, vel, tor, temp16 / 10.0


if __name__ == "__main__":
    # Minimal manual smoke test: opens the port, enables the motor, and
    # prints feedback with zero torque. Edit PORT / MOTOR_ID for your setup.
    PORT = "COM8"
    MOTOR_ID = 1

    bus = ATCanBus(PORT)
    try:
        set_run_mode_mit(bus, MOTOR_ID)
        enable(bus, MOTOR_ID)
        print("Reading feedback with zero torque, Ctrl+C to stop...")
        while True:
            resp = control(bus, MOTOR_ID, torque=0.0)
            fb = decode_feedback(resp)
            if fb:
                pos, vel, tor, temp = fb
                print(f"pos={pos:+.3f} rad  vel={vel:+.3f} rad/s  torque={tor:+.3f} Nm  temp={temp:.1f}C")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop(bus, MOTOR_ID)
        bus.shutdown()
