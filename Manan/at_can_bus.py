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
PARAM_RUN_MODE = 0x7005    # 0=MIT/operation, 1=position, 2=velocity, 3=current
PARAM_SPD_REF = 0x700A     # target velocity, native Velocity mode
PARAM_LIMIT_CUR = 0x7018   # max current
PARAM_ACC_RAD = 0x7022     # acceleration, native Velocity mode

RUN_MODE_MIT = 0
RUN_MODE_POSITION = 1
RUN_MODE_VELOCITY = 2
RUN_MODE_CURRENT = 3


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


def set_run_mode_mit(bus, motor_id, host_id=HOST_ID, timeout=0.1):
    """Reset to MIT/operation mode (0) - required before sending control()
    frames if the motor was previously left in Velocity/Current mode."""
    return write_param_u8(bus, motor_id, PARAM_RUN_MODE, RUN_MODE_MIT, host_id, timeout)


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
