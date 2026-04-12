from keyboard_control import inverse_kinematics
import math

# Test a known reachable point
angles, err = inverse_kinematics(0.10, 0.0, 0.20)
if angles:
    for k, v in angles.items():
        print(f"{k}: {math.degrees(v):.1f}°")
else:
    print(err)