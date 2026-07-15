"""
test_scripts.py

Test code for at_can_bus.py, encoder_feedback.py, and motor_control.py.

Two layers:

1. Offline unit tests (default, no hardware needed) - verify the AT-frame
   framing and the fixed-point encode/decode math are correct, using a
   fake serial port so nothing actually opens a COM/tty device. Run with:

       python test_scripts.py

2. Interactive hardware checks (opt-in, needs the real adapter + motor
   plugged in) - exercises encoder_feedback.py and motor_control.py
   against real hardware, with confirmation prompts before anything
   moves. Run with:

       python test_scripts.py --hardware --port COM8 --motor-id 1

Safety: the hardware section follows the same precautions as the rest of
this project (see ../README.md "Safety notes") - it asks for confirmation
before enabling the motor or commanding any motion, and always ramps
down + stops before exiting.
"""

import argparse
import math
import struct
import sys
import time
import unittest
from unittest import mock

import can

import at_can_bus as atc
import motor_control as mc


# ---------------------------------------------------------------------------
# 1. Offline unit tests - no hardware required
# ---------------------------------------------------------------------------

class FakeSerial:
    """Minimal stand-in for serial.Serial: an in-memory loopback buffer so
    ATCanBus can be exercised without a real port. Anything written with
    .write() can be read back with .read()."""

    def __init__(self, *args, **kwargs):
        self._buf = bytearray()

    def write(self, data):
        self._buf += data
        return len(data)

    def read(self, size=1):
        chunk = bytes(self._buf[:size])
        self._buf = self._buf[size:]
        return chunk

    def close(self):
        pass


class TestFixedPointMath(unittest.TestCase):
    """float_to_uint / uint_to_float should round-trip within one LSB, and
    always clamp to range instead of overflowing/wrapping."""

    def test_round_trip_midpoint(self):
        x = uint_x = atc.uint_to_float(
            atc.float_to_uint(0.0, atc.P_MIN, atc.P_MAX, 16), atc.P_MIN, atc.P_MAX, 16
        )
        self.assertAlmostEqual(x, 0.0, delta=0.001)

    def test_round_trip_endpoints(self):
        for x_min, x_max in [(atc.P_MIN, atc.P_MAX), (atc.V_MIN, atc.V_MAX),
                              (atc.T_MIN, atc.T_MAX), (atc.KP_MIN, atc.KP_MAX)]:
            lo = atc.uint_to_float(atc.float_to_uint(x_min, x_min, x_max, 16), x_min, x_max, 16)
            hi = atc.uint_to_float(atc.float_to_uint(x_max, x_min, x_max, 16), x_min, x_max, 16)
            self.assertAlmostEqual(lo, x_min, delta=0.001)
            self.assertAlmostEqual(hi, x_max, delta=0.001)

    def test_clamps_out_of_range(self):
        # A value far outside range should clamp to the max encodable value,
        # not overflow into garbage.
        over = atc.float_to_uint(atc.P_MAX + 100, atc.P_MIN, atc.P_MAX, 16)
        under = atc.float_to_uint(atc.P_MIN - 100, atc.P_MIN, atc.P_MAX, 16)
        self.assertEqual(over, (1 << 16) - 1)
        self.assertEqual(under, 0)


class TestMakeId(unittest.TestCase):
    """make_id must pack [mode:5][data16:16][motor_id:8] then shift left 3
    bits - the adapter hardware-register quirk documented in
    ../docs/protocol_notes.md. Get the shift direction wrong and every
    frame is silently addressed to the wrong mode/motor."""

    def test_bit_layout_and_shift(self):
        tx_id = atc.make_id(mode=1, data16=0xFD, motor_id=7)
        # un-shift to recover the true id and check the fields
        true_id = tx_id >> 3
        self.assertEqual(tx_id & 0b111, 0)  # bottom 3 bits must be zero (the shift)
        self.assertEqual(true_id & 0xFF, 7)              # motor_id
        self.assertEqual((true_id >> 8) & 0xFFFF, 0xFD)  # data16
        self.assertEqual((true_id >> 24) & 0x1F, 1)      # mode

    def test_distinct_motor_ids_distinct_frames(self):
        id_a = atc.make_id(1, atc.HOST_ID, motor_id=1)
        id_b = atc.make_id(1, atc.HOST_ID, motor_id=2)
        self.assertNotEqual(id_a, id_b)


class TestDecodeFeedback(unittest.TestCase):
    """decode_feedback should invert control()'s packing for a synthetic
    mode-2 reply, and return None for malformed frames instead of raising."""

    def test_decodes_known_values(self):
        import struct
        p_int = atc.float_to_uint(1.0, atc.P_MIN, atc.P_MAX, 16)
        v_int = atc.float_to_uint(2.0, atc.V_MIN, atc.V_MAX, 16)
        t_int = atc.float_to_uint(0.5, atc.T_MIN, atc.T_MAX, 16)
        temp16 = 250  # 25.0 C
        data = struct.pack('>HHHH', p_int, v_int, t_int, temp16)
        msg = can.Message(arbitration_id=0, data=data, is_extended_id=True)

        pos, vel, tor, temp = atc.decode_feedback(msg)
        self.assertAlmostEqual(pos, 1.0, delta=0.01)
        self.assertAlmostEqual(vel, 2.0, delta=0.01)
        self.assertAlmostEqual(tor, 0.5, delta=0.01)
        self.assertAlmostEqual(temp, 25.0, delta=0.01)

    def test_none_on_short_frame(self):
        msg = can.Message(arbitration_id=0, data=b'\x00\x01', is_extended_id=True)
        self.assertIsNone(atc.decode_feedback(msg))
        self.assertIsNone(atc.decode_feedback(None))


class TestATCanBusFraming(unittest.TestCase):
    """End-to-end: send() should write a correctly-framed AT packet, and
    recv() should parse an equivalent packet back into a can.Message with
    the same id/data - using a fake in-memory serial port, no hardware."""

    def setUp(self):
        patcher = mock.patch.object(atc.serial, 'Serial', FakeSerial)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.bus = atc.ATCanBus('FAKE')
        self.addCleanup(self.bus.shutdown)

    def test_send_frame_format(self):
        msg = can.Message(arbitration_id=0x12345678, data=b'\x01\x02\x03', is_extended_id=True)
        self.bus.send(msg)
        raw = bytes(self.bus.ser._buf)
        self.assertTrue(raw.startswith(b'AT'))
        self.assertTrue(raw.endswith(b'\r\n'))
        self.assertEqual(raw[2:6], (0x12345678).to_bytes(4, 'big'))
        self.assertEqual(raw[6], 3)  # DLC
        self.assertEqual(raw[7:10], b'\x01\x02\x03')

    def test_send_then_recv_round_trip(self):
        msg = can.Message(arbitration_id=0xAABBCCDD, data=b'\xde\xad\xbe\xef', is_extended_id=True)
        self.bus.send(msg)
        # loop the write buffer back as if it were read from the port
        looped = bytes(self.bus.ser._buf)
        self.bus.ser._buf = bytearray(looped)
        got = self.bus.recv(timeout=0.5)
        self.assertIsNotNone(got)
        self.assertEqual(got.arbitration_id, 0xAABBCCDD)
        self.assertEqual(bytes(got.data), b'\xde\xad\xbe\xef')

    def test_recv_returns_none_on_no_data(self):
        got = self.bus.recv(timeout=0.05)
        self.assertIsNone(got)


class TestNativeModeFraming(unittest.TestCase):
    """Native Current/Velocity/Location mode helpers should all go through
    write_param_f32 with the right index/value, clamp to the documented
    ranges instead of overflowing, and set_run_mode_* should write the
    right run_mode byte via write_param_u8."""

    def setUp(self):
        self.sent = []
        patcher = mock.patch.object(
            atc, 'write_param_f32',
            side_effect=lambda bus, motor_id, param_id, value, host_id=atc.HOST_ID, timeout=0.1:
                self.sent.append((param_id, value)),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_set_current_uses_iq_ref(self):
        atc.set_current(None, 1, 5.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_IQ_REF, 5.0))

    def test_set_current_clamps_range(self):
        atc.set_current(None, 1, 999.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_IQ_REF, atc.IQ_MAX))
        atc.set_current(None, 1, -999.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_IQ_REF, atc.IQ_MIN))

    def test_set_velocity_uses_spd_ref_and_clamps(self):
        atc.set_velocity(None, 1, 40.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_SPD_REF, atc.V_MAX))

    def test_set_velocity_limit_cur_clamps(self):
        atc.set_velocity_limit_cur(None, 1, -5.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_LIMIT_CUR, atc.CUR_LIMIT_MIN))
        atc.set_velocity_limit_cur(None, 1, 50.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_LIMIT_CUR, atc.CUR_LIMIT_MAX))

    def test_set_location_uses_loc_ref_and_clamps(self):
        atc.set_location(None, 1, 20.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_LOC_REF, atc.P_MAX))

    def test_position_pp_profile_writes_both_params(self):
        atc.set_position_pp_profile(None, 1, vel_max=10.0, acc_set=20.0)
        self.assertIn((atc.PARAM_VEL_MAX, 10.0), self.sent)
        self.assertIn((atc.PARAM_ACC_SET, 20.0), self.sent)

    def test_position_csp_limit_spd(self):
        atc.set_position_csp_limit_spd(None, 1, 8.0)
        self.assertEqual(self.sent[-1], (atc.PARAM_LIMIT_SPD, 8.0))


class TestRunModeSwitching(unittest.TestCase):
    """set_run_mode_* wrappers should each write the correct run_mode value
    via write_param_u8 (mode 18, single byte)."""

    def setUp(self):
        self.sent = []
        patcher = mock.patch.object(
            atc, 'write_param_u8',
            side_effect=lambda bus, motor_id, param_id, value, host_id=atc.HOST_ID, timeout=0.1:
                self.sent.append((param_id, value)),
        )
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_mit(self):
        atc.set_run_mode_mit(None, 1)
        self.assertEqual(self.sent[-1], (atc.PARAM_RUN_MODE, atc.RUN_MODE_MIT))

    def test_current(self):
        atc.set_run_mode_current(None, 1)
        self.assertEqual(self.sent[-1], (atc.PARAM_RUN_MODE, atc.RUN_MODE_CURRENT))

    def test_velocity(self):
        atc.set_run_mode_velocity(None, 1)
        self.assertEqual(self.sent[-1], (atc.PARAM_RUN_MODE, atc.RUN_MODE_VELOCITY))

    def test_position_pp(self):
        atc.set_run_mode_position_pp(None, 1)
        self.assertEqual(self.sent[-1], (atc.PARAM_RUN_MODE, atc.RUN_MODE_POSITION_PP))

    def test_position_csp(self):
        atc.set_run_mode_position_csp(None, 1)
        self.assertEqual(self.sent[-1], (atc.PARAM_RUN_MODE, atc.RUN_MODE_POSITION_CSP))


class TestWriteParamF32Framing(unittest.TestCase):
    """write_param_f32 must lay out bytes exactly per the protocol: u16
    index, 2 zero bytes, f32 little-endian value - and actually go out
    over the bus."""

    def setUp(self):
        patcher = mock.patch.object(atc.serial, 'Serial', FakeSerial)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.bus = atc.ATCanBus('FAKE')
        self.addCleanup(self.bus.shutdown)

    def test_byte_layout(self):
        self.bus.recv = lambda timeout=None: None  # no motor to reply
        atc.write_param_f32(self.bus, 1, atc.PARAM_SPD_REF, 5.0)
        raw = bytes(self.bus.ser._buf)
        dlc = raw[6]
        data = raw[7:7 + dlc]
        self.assertEqual(dlc, 8)
        idx, zero, val = struct.unpack('<HHf', data)
        self.assertEqual(idx, atc.PARAM_SPD_REF)
        self.assertEqual(zero, 0)
        self.assertAlmostEqual(val, 5.0, places=5)


class TestNativeModeCallOrder(unittest.TestCase):
    """Regression test for a real bug: the RobStride manual's precautions
    section says "Do not switch the control mode when the joint is
    running. If you need to switch, send the command to stop the
    operation before switching." motor_control.main() enables the motor
    (in MIT mode) before dispatching to any subcommand, so each native
    run_*_mode() helper MUST call stop() before its set_run_mode_*() call
    - otherwise the mode switch (and everything depending on it, like
    spd_ref) is silently ignored and the motor never leaves MIT mode."""

    def setUp(self):
        self.calls = []

        def record(name):
            def _fn(*a, **kw):
                self.calls.append(name)
            return _fn

        def record_and_raise_once(name):
            state = {'raised': False}

            def _fn(*a, **kw):
                self.calls.append(name)
                if not state['raised']:
                    state['raised'] = True
                    raise KeyboardInterrupt
            return _fn

        patches = [
            mock.patch.object(mc, 'stop', side_effect=record('stop')),
            mock.patch.object(mc, 'enable', side_effect=record('enable')),
            mock.patch.object(mc, 'set_run_mode_current', side_effect=record('set_run_mode_current')),
            mock.patch.object(mc, 'set_run_mode_velocity', side_effect=record('set_run_mode_velocity')),
            mock.patch.object(mc, 'set_run_mode_position_pp', side_effect=record('set_run_mode_position_pp')),
            mock.patch.object(mc, 'set_run_mode_position_csp', side_effect=record('set_run_mode_position_csp')),
            mock.patch.object(mc, 'set_velocity_limit_cur', side_effect=record('set_velocity_limit_cur')),
            mock.patch.object(mc, 'set_velocity_accel', side_effect=record('set_velocity_accel')),
            mock.patch.object(mc, 'set_position_pp_profile', side_effect=record('set_position_pp_profile')),
            mock.patch.object(mc, 'set_position_csp_limit_spd', side_effect=record('set_position_csp_limit_spd')),
            mock.patch.object(mc, 'set_current', side_effect=record_and_raise_once('set_current')),
            mock.patch.object(mc, 'set_velocity', side_effect=record_and_raise_once('set_velocity')),
            mock.patch.object(mc, 'set_location', side_effect=record_and_raise_once('set_location')),
            mock.patch.object(mc, 'decode_feedback', return_value=None),
            mock.patch.object(mc.time, 'sleep', return_value=None),
        ]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)

    def test_current_mode_stops_before_switching(self):
        mc.run_current_mode(None, 1, target_iq=2.0)
        self.assertEqual(self.calls[:3], ['stop', 'set_run_mode_current', 'enable'])

    def test_velocity_mode_stops_before_switching(self):
        mc.run_velocity_native_mode(None, 1, target_vel=5.0)
        self.assertEqual(self.calls[:3], ['stop', 'set_run_mode_velocity', 'enable'])

    def test_location_mode_pp_stops_before_switching(self):
        mc.run_location_mode(None, 1, target_rad=1.0, mode_type='pp')
        self.assertEqual(self.calls[:3], ['stop', 'set_run_mode_position_pp', 'enable'])

    def test_location_mode_csp_stops_before_switching(self):
        mc.run_location_mode(None, 1, target_rad=1.0, mode_type='csp')
        self.assertEqual(self.calls[:3], ['stop', 'set_run_mode_position_csp', 'enable'])


def run_offline_tests():
    loader = unittest.TestLoader()
    suite = unittest.TestSuite([
        loader.loadTestsFromTestCase(TestFixedPointMath),
        loader.loadTestsFromTestCase(TestMakeId),
        loader.loadTestsFromTestCase(TestDecodeFeedback),
        loader.loadTestsFromTestCase(TestATCanBusFraming),
        loader.loadTestsFromTestCase(TestNativeModeFraming),
        loader.loadTestsFromTestCase(TestRunModeSwitching),
        loader.loadTestsFromTestCase(TestWriteParamF32Framing),
        loader.loadTestsFromTestCase(TestNativeModeCallOrder),
    ])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return result.wasSuccessful()


# ---------------------------------------------------------------------------
# 2. Interactive hardware checks - needs the real adapter + motor
# ---------------------------------------------------------------------------

def confirm(prompt):
    return input(f"{prompt} [y/N]: ").strip().lower() == 'y'


def hardware_checks(port, motor_id):
    print("\n=== HARDWARE CHECKS ===")
    print(f"Port: {port}   Motor id: {motor_id}")
    print("Make sure the rod/shaft has full clearance to move before continuing.\n")

    if not confirm("Open the serial port and talk to the adapter now?"):
        print("Skipped.")
        return

    bus = atc.ATCanBus(port)
    try:
        print("Waiting for the adapter/motor to settle (power-cycle the motor now if it "
              "was already on)...")
        time.sleep(3)

        atc.set_run_mode_mit(bus, motor_id)
        time.sleep(0.1)

        print("\n[1/3] Enable + read feedback once (zero torque)...")
        atc.enable(bus, motor_id)
        time.sleep(0.1)
        resp = atc.control(bus, motor_id, torque=0.0)
        fb = atc.decode_feedback(resp)
        if fb:
            pos, vel, tor, temp = fb
            print(f"  OK - pos={pos:+.3f}rad vel={vel:+.3f}rad/s tor={tor:+.3f}Nm temp={temp:.1f}C")
        else:
            print("  No feedback received - check port/motor id/wiring before continuing.")
            return

        print("\n[2/3] Read encoder feedback for 3 seconds (nudge the shaft by hand to confirm sign)...")
        if confirm("Proceed?"):
            t0 = time.time()
            while time.time() - t0 < 3.0:
                resp = atc.control(bus, motor_id, torque=0.0)
                fb = atc.decode_feedback(resp)
                if fb:
                    pos, vel, tor, temp = fb
                    print(f"  pos={math.degrees(pos):+7.1f}deg  vel={vel:+.3f}rad/s")
                time.sleep(0.1)

        print("\n[3/3] Tiny, damped position nudge (kp=5, kd=1, target = current + 0.05 rad)...")
        if confirm("This will move the shaft slightly. Proceed?"):
            resp = atc.control(bus, motor_id, torque=0.0)
            fb = atc.decode_feedback(resp)
            start_pos = fb[0] if fb else 0.0
            target = start_pos + 0.05
            t0 = time.time()
            while time.time() - t0 < 2.0:
                resp = atc.control(bus, motor_id, position=target, velocity=0.0, kp=5.0, kd=1.0, torque=0.0)
                fb = atc.decode_feedback(resp)
                if fb:
                    pos, vel, tor, temp = fb
                    print(f"  target={math.degrees(target):+7.1f}deg  pos={math.degrees(pos):+7.1f}deg  "
                          f"vel={vel:+.3f}rad/s  tor={tor:+.3f}Nm")
                time.sleep(0.02)
            print("  Ramping down...")
            for _ in range(30):
                atc.control(bus, motor_id, position=0.0, velocity=0.0, kp=0.0, kd=2.0, torque=0.0)
                time.sleep(0.02)

        print("\nAll requested hardware checks complete.")
    finally:
        atc.stop(bus, motor_id)
        bus.shutdown()
        print("Motor stopped, port closed. Safe to power off.")


def main():
    parser = argparse.ArgumentParser(description="Test at_can_bus.py / encoder_feedback.py / motor_control.py")
    parser.add_argument('--hardware', action='store_true',
                         help="Also run interactive hardware checks (needs real adapter + motor)")
    parser.add_argument('--port', default='COM8', help="Serial port for hardware checks")
    parser.add_argument('--motor-id', type=int, default=1, help="CAN motor id for hardware checks")
    args = parser.parse_args()

    print("Running offline unit tests (no hardware needed)...\n")
    ok = run_offline_tests()

    if args.hardware:
        hardware_checks(args.port, args.motor_id)

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
