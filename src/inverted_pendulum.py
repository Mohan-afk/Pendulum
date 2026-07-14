"""
Inverted pendulum (rotary, direct-drive) test script.

Stages, run one at a time by editing STAGE below:
  'zero'    - calibrate: hang rod straight down, this sets that as 0 rad
  'swing'   - free-swing test, NO torque, just watching natural dynamics
  'balance' - slow trajectory up to pi (180deg), then hold there

Physics check (edit ROD_MASS_KG / ROD_LENGTH_M for your actual rod):
  gravity's destabilizing torque at top ~= m*g*(L/2)
  balance requires kp > that value - with a light aluminum rod this
  threshold is usually under 1 Nm/rad, and this motor's kp goes up to
  500, so there's a lot of margin. Still start conservative.

Edit PORT below:
  Windows: 'COM6'
  Linux:   '/dev/ttyUSB0'
"""
import sys
sys.path.insert(0, '.')
from at_can_bus import ATCanBus
import can, time, struct, math

PORT = 'COM6'   # <-- CHANGE THIS
MOTOR = 'motor1'  # <-- change to 'motor2' if that's the one you wired up

STAGE = 'zero'   # 'zero' | 'swing' | 'balance'

# --- physics estimate, for reference / sanity check only ---
ROD_MASS_KG = 0.18
ROD_LENGTH_M = 0.5
G = 9.81
GRAVITY_TORQUE_AT_TOP = ROD_MASS_KG * G * (ROD_LENGTH_M / 2)
print(f"(estimated gravity torque at inverted position: {GRAVITY_TORQUE_AT_TOP:.3f} Nm - "
      f"kp must exceed this to balance)")

HOST_ID = 0xFD
MOTOR_IDS = {'motor1': 127, 'motor2': 2}

P_MIN, P_MAX = -12.57, 12.57
V_MIN, V_MAX = -33.0, 33.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0
T_MIN, T_MAX = -14.0, 14.0

PARAM_RUN_MODE = 0x7005

bus = ATCanBus(PORT)

def float_to_uint(x, x_min, x_max, bits):
    x = max(x_min, min(x_max, x))
    span = x_max - x_min
    return int((x - x_min) * ((1 << bits) - 1) / span)

def uint_to_float(x, x_min, x_max, bits):
    span = x_max - x_min
    return float(x) / ((1 << bits) - 1) * span + x_min

def make_id(mode, data16, motor_id):
    true_id = (mode << 24) | (data16 << 8) | motor_id
    return true_id << 3

def enable(name):
    mid = MOTOR_IDS[name]
    tx_id = make_id(3, HOST_ID, mid)
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True))
    return bus.recv(timeout=0.1)

def stop(name):
    mid = MOTOR_IDS[name]
    tx_id = make_id(4, HOST_ID, mid)
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True))
    return bus.recv(timeout=0.1)

def set_zero(name):
    """Communication type 6: set current position as mechanical zero."""
    mid = MOTOR_IDS[name]
    tx_id = make_id(6, HOST_ID, mid)
    data = bytes([1, 0, 0, 0, 0, 0, 0, 0])
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=0.1)

def control(name, position, velocity, kp, kd, torque):
    mid = MOTOR_IDS[name]
    t_int = float_to_uint(torque, T_MIN, T_MAX, 16)
    tx_id = make_id(1, t_int, mid)
    p_int = float_to_uint(position, P_MIN, P_MAX, 16)
    v_int = float_to_uint(velocity, V_MIN, V_MAX, 16)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16)
    data = struct.pack('>HHHH', p_int, v_int, kp_int, kd_int)
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=0.05)

def write_param_u8(name, param_id, value):
    mid = MOTOR_IDS[name]
    tx_id = make_id(18, HOST_ID, mid)
    data = struct.pack('<HH', param_id, 0) + bytes([value]) + bytes(3)
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=0.1)

def decode_feedback(resp):
    p16, v16, t16, temp16 = struct.unpack('>HHHH', resp.data)
    pos = uint_to_float(p16, P_MIN, P_MAX, 16)
    vel = uint_to_float(v16, V_MIN, V_MAX, 16)
    tor = uint_to_float(t16, T_MIN, T_MAX, 16)
    return pos, vel, tor, temp16 / 10.0

print(f"Power cycle {MOTOR}, waiting 3s...")
time.sleep(3)

write_param_u8(MOTOR, PARAM_RUN_MODE, 0)
time.sleep(0.1)

if STAGE == 'zero':
    print("\n=== ZERO CALIBRATION ===")
    print("Hang the rod so it points straight DOWN (the stable equilibrium).")
    input("Press Enter once it's hanging still, straight down...")
    r = set_zero(MOTOR)
    print("Zero set. Current position should now read ~0:")
    r = enable(MOTOR)
    time.sleep(0.1)
    r = control(MOTOR, 0.0, 0.0, 0.0, 0.0, 0.0)
    if r:
        pos, vel, tor, temp = decode_feedback(r)
        print(f"  pos={pos:+.4f}rad (should be near 0)")
    stop(MOTOR)
    bus.shutdown()
    print("Done. Now set STAGE='swing' or STAGE='balance' and rerun.")
    sys.exit()

if STAGE == 'swing':
    print("\n=== FREE SWING TEST (no torque, just watching) ===")
    print("Nudge the rod by hand, let it swing. Ctrl+C to stop.\n")
    r = enable(MOTOR)
    try:
        while True:
            r = control(MOTOR, 0.0, 0.0, 0.0, 0.0, 0.0)  # zero torque, just reading
            if r:
                pos, vel, tor, temp = decode_feedback(r)
                print(f"pos={pos:+.3f}rad ({math.degrees(pos):+.1f}deg)  vel={vel:+.3f}rad/s")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopping...")
    stop(MOTOR)
    bus.shutdown()
    print("Done.")
    sys.exit()

if STAGE == 'balance':
    print("\n=== SWING UP + BALANCE AT TOP ===")
    print("Assumes zero calibration already done (rod was hanging down = 0).")
    print("Will slowly move to pi rad (180deg = inverted) over 4 seconds,")
    print("then hold there. Ctrl+C aborts immediately.\n")

    r = enable(MOTOR)
    time.sleep(0.1)

    KP_HOLD = 15.0   # comfortably above the ~0.4Nm/rad gravity threshold
    KD_HOLD = 1.0

    DURATION = 4.0
    TARGET = math.pi

    t0 = time.time()
    try:
        # smooth ramp from 0 to pi
        while True:
            t = time.time() - t0
            if t < DURATION:
                # simple smoothstep (not just linear) for gentler accel/decel
                frac = t / DURATION
                smooth = frac * frac * (3 - 2 * frac)
                target_pos = smooth * TARGET
                kp, kd = KP_HOLD, KD_HOLD
            else:
                target_pos = TARGET
                kp, kd = KP_HOLD, KD_HOLD

            r = control(MOTOR, position=target_pos, velocity=0.0, kp=kp, kd=kd, torque=0.0)
            if r:
                pos, vel, tor, temp = decode_feedback(r)
                print(f"t={t:4.1f}s target={math.degrees(target_pos):6.1f}deg  "
                      f"pos={math.degrees(pos):+7.1f}deg  vel={vel:+.3f}rad/s  tor={tor:+.3f}Nm")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopping...")

    print("Ramping down...")
    for _ in range(30):
        control(MOTOR, 0.0, 0.0, 0.0, 2.0, 0.0)
        time.sleep(0.02)
    stop(MOTOR)
    bus.shutdown()
    print("Done. Safe to power off.")
