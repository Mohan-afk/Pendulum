"""
Soft balance at theta=pi (inverted), using our OWN feedback law computed
in software, sent as pure torque (kp=0, kd=0 in the CAN frame - the
firmware's stiff spring/damper is NOT used here).

Control law each cycle:
  tau = gravity_comp(theta) - K*(theta - pi) - D*theta_dot

gravity_comp cancels gravity entirely, so the rod would float at ANY
angle without it. -K*(theta-pi) is what makes pi special - a soft virtual
spring. Push the rod and it should drift back gently, not snap back like
a stiff position hold.

Assumes zero calibration already done (rod hanging straight down = 0).

Edit PORT below:
  Windows: 'COM6'
  Linux:   '/dev/ttyUSB0'
"""
import sys
sys.path.insert(0, '.')
from at_can_bus import ATCanBus
import can, time, struct, math

PORT = 'COM6'   # <-- CHANGE THIS
MOTOR_ID = 127

# --- physics / gains - tune these ---
ROD_MASS_KG = 0.18
ROD_LENGTH_M = 0.5
G = 9.81
GRAVITY_COEFF = ROD_MASS_KG * G * (ROD_LENGTH_M / 2)  # peak gravity torque, Nm

K_SOFT = 2    # restoring "stiffness" toward pi - much softer than a position-hold kp
D_SOFT = 0.2    # damping
KI_SOFT = 0.01  # integral gain - kills steady-state offset, doesn't affect softness
I_MAX = 1.0     # anti-windup clamp on the integral term's torque contribution (Nm)

SWING_DURATION = 4.0   # seconds to get from 0 to pi before handing off to the balance law

HOST_ID = 0xFD
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

def enable():
    tx_id = make_id(3, HOST_ID, MOTOR_ID)
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True))
    return bus.recv(timeout=0.1)

def stop():
    tx_id = make_id(4, HOST_ID, MOTOR_ID)
    bus.send(can.Message(arbitration_id=tx_id, data=bytes(8), is_extended_id=True))
    return bus.recv(timeout=0.1)

def control(position, velocity, kp, kd, torque):
    t_int = float_to_uint(torque, T_MIN, T_MAX, 16)
    tx_id = make_id(1, t_int, MOTOR_ID)
    p_int = float_to_uint(position, P_MIN, P_MAX, 16)
    v_int = float_to_uint(velocity, V_MIN, V_MAX, 16)
    kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 16)
    kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 16)
    data = struct.pack('>HHHH', p_int, v_int, kp_int, kd_int)
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=0.05)

def write_param_u8(param_id, value):
    tx_id = make_id(18, HOST_ID, MOTOR_ID)
    data = struct.pack('<HH', param_id, 0) + bytes([value]) + bytes(3)
    bus.send(can.Message(arbitration_id=tx_id, data=data, is_extended_id=True))
    return bus.recv(timeout=0.1)

def decode_feedback(resp):
    p16, v16, t16, temp16 = struct.unpack('>HHHH', resp.data)
    pos = uint_to_float(p16, P_MIN, P_MAX, 16)
    vel = uint_to_float(v16, V_MIN, V_MAX, 16)
    tor = uint_to_float(t16, T_MIN, T_MAX, 16)
    return pos, vel, tor, temp16 / 10.0

print("Power cycle motor, waiting 3s...")
time.sleep(3)

write_param_u8(PARAM_RUN_MODE, 0)
time.sleep(0.1)

print("Enabling...")
enable()
time.sleep(0.1)

print(f"Swinging up to pi over {SWING_DURATION}s using firmware position hold...")
t0 = time.time()
try:
    while True:
        t = time.time() - t0
        if t < SWING_DURATION:
            frac = t / SWING_DURATION
            smooth = frac * frac * (3 - 2 * frac)
            target = smooth * math.pi
            r = control(target, 0.0, 15.0, 1.0, 0.0)
        else:
            break
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\nAborted during swing-up.")
    stop()
    bus.shutdown()
    sys.exit()

print("\nSwitching to SOFT feedback balance at pi. Try pushing the rod!")
print("Ctrl+C to stop.\n")

theta, theta_dot = math.pi, 0.0
tau = 0.0
integral = 0.0
last_time = time.time()
try:
    while True:
        r = control(0.0, 0.0, 0.0, 0.0, tau)
        if r:
            now = time.time()
            dt = now - last_time
            last_time = now
            theta, theta_dot, tor_actual, temp = decode_feedback(r)
            gravity_comp = GRAVITY_COEFF * math.sin(theta)
            error = theta - math.pi
            integral += error * dt
            integral_torque = KI_SOFT * integral
            # anti-windup: clamp the integral contribution so it can't run away
            integral_torque = max(-I_MAX, min(I_MAX, integral_torque))
            if abs(integral_torque) >= I_MAX:
                integral = integral_torque / KI_SOFT  # back-calculate to stop windup
            tau = gravity_comp - K_SOFT * error - D_SOFT * theta_dot - integral_torque
            print(f"theta={math.degrees(theta):+7.1f}deg  vel={theta_dot:+.3f}rad/s  "
                  f"tau_cmd={tau:+.3f}Nm  tau_actual={tor_actual:+.3f}Nm  "
                  f"grav={gravity_comp:+.3f}Nm  I={integral_torque:+.3f}Nm")
        time.sleep(0.02)
except KeyboardInterrupt:
    print("\nStopping...")

print("Ramping down...")
for _ in range(30):
    control(0.0, 0.0, 0.0, 2.0, 0.0)
    time.sleep(0.02)
stop()
bus.shutdown()
print("Done. Safe to power off.")
