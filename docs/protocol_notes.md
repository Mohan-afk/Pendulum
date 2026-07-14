# RobStride Motor Control — How It All Fits Together

## 1. The Hardware

- **Motors**: RobStride RS00, CAN bus @ 1Mbps, daisy-chained (CAN_H→CAN_H, CAN_L→CAN_L, GND common). Each motor has a unique **CAN ID** (yours: motor1=`1`, motor2=`2`).
- **Adapter**: RobStride USB-to-CAN module. USB-C to the Jetson/PC, 3-pin screw terminal (GND/CAN_H/CAN_L) to the motors. Internally: CH340 (USB↔serial chip) + GD32 MCU (does the actual CAN framing).
- **DIP switches on adapter**: SW1=OFF (else it's stuck in boot mode), SW2=ON (enables 120Ω bus termination).

The adapter doesn't speak raw CAN over USB — it speaks its own **serial "AT protocol"** at 921600 baud, and translates that into real CAN frames on the wire.

## 2. The AT Serial Protocol

Every command to the adapter, and every reply from it, is a serial frame:

```
41 54  [4-byte CAN ID]  [1-byte DLC]  [0-8 bytes data]  0D 0A
"AT"                                                    "\r\n"
```

`at_can_bus.py` is the Python driver that speaks this. It:
- Opens the serial port at 921600 baud (`ATCanBus.__init__`)
- `send()`: takes a `can.Message`, wraps its arbitration ID + data into this AT frame, writes it to the serial port
- `recv()`: reads raw bytes, finds the `AT ... \r\n` markers, extracts the CAN ID and data, hands back a `can.Message`

This driver exists because the adapter's CH340 chip doesn't show up as a standard SocketCAN device on Linux — it's just a serial port that happens to carry CAN frames in this custom wrapper.

## 3. The Real Discovery: the CAN ID Isn't What It Looks Like

This is the part that cost the most time. The 4-byte "CAN ID" field isn't the logical CAN ID directly — it's a **raw hardware register value**, which is the true ID **shifted left by 3 bits**. You have to shift right by 3 before it makes sense, and shift left by 3 when constructing one to send.

The true 29-bit ID (after un-shifting) breaks down as:

```
[ mode: 5 bits ] [ data: 16 bits ] [ motor_id: 8 bits ]
```

- **mode** = what kind of message this is (see below)
- **data** = usually the *host ID* (`0xFD`, a fixed convention), except in control frames where it's repurposed to carry the torque value
- **motor_id** = which motor this is addressed to (or, in a reply, which motor it's *from*)

## 4. The Message Types ("modes") That Matter

| mode | Name | Purpose |
|---|---|---|
| 1 | Control | Send position/velocity/kp/kd/torque targets |
| 2 | Feedback | Motor's reply: current position/velocity/torque/temp |
| 3 | Enable | Turn on the motor's control loop |
| 4 | Stop | Disable |
| 17 | Param Read | Read a named parameter (e.g. current run mode) |
| 18 | Param Write | Set a named parameter (e.g. run mode, velocity setpoint) |

**Mode 1 (control) is special**: instead of just using the 8 data bytes, it also hides the **torque** value inside the ID's "data" field. The 8 data bytes carry position, velocity, kp, kd (each a 16-bit fixed-point number, big-endian, in that order).

## 5. Parameters (used with mode 18, param write)

Each motor has a table of named registers, addressed by a 16-bit index. The ones we use:

| Index | Name | Meaning |
|---|---|---|
| `0x7005` | `run_mode` | 0=operation/MIT control, 1=position, 2=velocity, 3=current |
| `0x700A` | `spd_ref` | Target velocity (used in native Velocity mode) |
| `0x7018` | `limit_cur` | Max current |
| `0x7022` | `acc_rad` | Acceleration (Velocity mode) |

**Important gotcha**: if a motor was ever put into a different `run_mode` (e.g. Velocity mode from an earlier test), it may ignore mode-1 control frames until you explicitly write `run_mode=0` again. Always do this before starting control, especially after switching between control styles.

## 6. What `robstride_official.py` Actually Does

1. **Reset run_mode to 0** on each motor (param write, mode 18) — clears any leftover state.
2. **Enable** each motor (mode 3).
3. **Loop**: repeatedly send a **mode 1 control frame** with your desired `position`, `velocity`, `kp`, `kd`, `torque`. The motor's own firmware runs the actual control law:

   ```
   torque_output = kp × (position_target − position_actual)
                 + kd × (velocity_target − velocity_actual)
                 + torque_feedforward
   ```

   So "velocity mode" here isn't a special firmware mode at all — it's just `kp=0`, so only the velocity error drives torque. Position holding is `kd`+`kp` with `velocity_target=0`. This one frame type covers position, velocity, and torque control depending on which gains you set to zero.
4. Each control frame gets back a **mode 2 feedback frame** — decoded into real position/velocity/torque/temperature.
5. **Shutdown**: ramp `kd` up with zero targets (damping to a stop) before sending the final `stop` (mode 4) — avoids a sudden jerk.

## 7. Why Two Motors on One Bus Just Works

Both motors listen on the same physical CAN_H/CAN_L wires. Each frame's `motor_id` field (the low byte of the true ID) tells only the intended motor to respond — motor1 ignores frames addressed to motor2's ID and vice versa. Their replies are distinguishable the same way: each motor stamps its own ID in its feedback frame.

## Summary Cheat Sheet

```
Host ID:        0xFD
Motor IDs:      motor1=1, motor2=2
Raw AT frame ID = (mode<<24 | data16<<8 | motor_id) << 3   <-- the shift matters!

mode 3  = enable
mode 4  = stop
mode 1  = control (torque in ID, pos/vel/kp/kd in 8 data bytes, big-endian u16 each)
mode 18 = write parameter (index + value in data bytes)

Before controlling: always set run_mode (0x7005) = 0 first.
```
