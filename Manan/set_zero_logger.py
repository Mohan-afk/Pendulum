"""
set_zero_logger.py

Interactive script to set the motor's mechanical zero position and log the event.
This is useful when a shaft is connected and you need to calibrate the 0 rad
reference point before running control scripts.
"""

import argparse
import time
import json
import os
import math
from datetime import datetime

from at_can_bus import (
    ATCanBus,
    set_zero,
    control,
    decode_feedback,
    set_run_mode_mit,
    enable,
    stop
)

PORT = 'COM8'
MOTOR_ID = 1

def log_zero_position(motor_id, port, pre_pos, post_pos, log_file="zero_calibration_log.json"):
    log_entry = {
        "timestamp": datetime.now().isoformat(),
        "motor_id": motor_id,
        "port": port,
        "event": "set_zero",
        "pre_calibration_pos_rad": pre_pos,
        "pre_calibration_pos_deg": round(math.degrees(pre_pos), 2) if pre_pos is not None else None,
        "post_calibration_pos_rad": post_pos,
        "post_calibration_pos_deg": round(math.degrees(post_pos), 2) if post_pos is not None else None,
        "description": "Mechanical zero position set and saved."
    }
    
    logs = {}
    if os.path.exists(log_file):
        try:
            with open(log_file, 'r') as f:
                loaded = json.load(f)
                if isinstance(loaded, dict):
                    logs = loaded
        except json.JSONDecodeError:
            pass
            
    key = f"{port}_{motor_id}"
    logs[key] = log_entry
    
    with open(log_file, 'w') as f:
        json.dump(logs, f, indent=4)
        
    print(f"Zero position configuration saved to {log_file}")

def main():
    parser = argparse.ArgumentParser(description="Set and log the motor's zero position.")
    parser.add_argument('--port', default=PORT, help=f"Serial port (default: {PORT})")
    parser.add_argument('--motor-id', type=int, default=MOTOR_ID, help=f"CAN motor id (default: {MOTOR_ID})")
    parser.add_argument('--log-file', default="zero_calibration_log.json", help="File to save the zero reference log")
    parser.add_argument('--clear-all', action='store_true', help="Clear all stored zero configurations and exit")
    args = parser.parse_args()

    if args.clear_all:
        if os.path.exists(args.log_file):
            os.remove(args.log_file)
            print(f"Cleared {args.log_file}")
        else:
            print(f"No log file found at {args.log_file}")
        return

    print(f"Opening {args.port}, motor id {args.motor_id}...")
    bus = ATCanBus(args.port)
    
    try:
        print("Waiting for the adapter/motor to settle...")
        time.sleep(3)
        
        # Ensure we are in MIT mode for control frames later
        set_run_mode_mit(bus, args.motor_id)
        time.sleep(0.1)

        print("\n" + "="*50)
        print("ZERO POSITION CALIBRATION")
        print("="*50)
        input("1. Hold the shaft/rod exactly at the physical position you want as 0 radians.\n2. Press Enter to set zero...")
        
        # Read the current position before zeroing
        print("\nReading position relative to previous zero...")
        enable(bus, args.motor_id)
        time.sleep(0.1)
        resp_before = control(bus, args.motor_id, torque=0.0)
        fb_before = decode_feedback(resp_before)
        pre_pos = fb_before[0] if fb_before else None
        stop(bus, args.motor_id)
        time.sleep(0.1)

        # Send set_zero command
        print("Sending set_zero command...")
        set_zero(bus, args.motor_id)
        time.sleep(0.5)
        
        # Enable motor to read feedback
        enable(bus, args.motor_id)
        time.sleep(0.1)
        
        # Verify with a zero-torque control frame
        resp_after = control(bus, args.motor_id, torque=0.0)
        fb_after = decode_feedback(resp_after)
        post_pos = fb_after[0] if fb_after else None
        if fb_after:
            print(f"Verification - Current Position: {post_pos:+.4f} rad ({math.degrees(post_pos):+.1f} deg)")
            if abs(post_pos) < 0.1:
                print("Success! Motor position is now near 0.")
            else:
                print("Warning: Motor position is not near 0. Please try again.")
        else:
            print("Verification failed: No feedback received from motor.")
            
        # Stop motor since we just enabled it to check feedback
        stop(bus, args.motor_id)
            
        # Log the calibration
        log_zero_position(args.motor_id, args.port, pre_pos, post_pos, args.log_file)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        bus.shutdown()
        print("\nDone.")

if __name__ == "__main__":
    main()
