import math

# Constants
VOLTAGE = 24  # V
POWER = 350  # W
ARM_WEIGHT = 3  # kg
GRAVITY = 9.81  # m/s^2
EFFICIENCY = 0.7  # Assumed efficiency of the system

# NEMA 17 typical specifications (you may need to adjust these based on your specific motors)
MOTOR_TORQUE = 0.5  # N·m (adjust this based on your specific NEMA 17 motor)
MOTOR_STEPS_PER_REV = 200

def calculate_max_torque(joint):
    # This is a simplified calculation and may need adjustment based on your arm's geometry
    if joint in ['J1', 'J2', 'J3']:
        return ARM_WEIGHT * GRAVITY * 0.7  # Assume the arm's center of mass is halfway along its length
    else:
        return ARM_WEIGHT * GRAVITY * 0.1  # Smaller torque for wrist joints

def calculate_time(angle, speed, acceleration, gear_ratio, joint):
    # Convert speed from steps/second to degrees/second
    speed_deg = speed * 1.8 / (gear_ratio * 16)  # 1.8 degrees per step, 1/16 microstepping
    
    # Convert acceleration from steps/second^2 to degrees/second^2
    accel_deg = acceleration * 1.8 / (gear_ratio * 16)
    
    # Calculate maximum torque required for this joint
    max_torque = calculate_max_torque(joint)
    
    # Adjust acceleration based on available torque
    available_torque = min(MOTOR_TORQUE * gear_ratio, POWER * EFFICIENCY / speed_deg)
    torque_factor = min(1, available_torque / max_torque)
    adjusted_accel = accel_deg * torque_factor
    
    # Calculate time to reach full speed
    time_to_full_speed = speed_deg / adjusted_accel
    
    # Calculate distance covered during acceleration
    distance_accel = 0.5 * adjusted_accel * time_to_full_speed**2
    
    if distance_accel * 2 < abs(angle):
        # If we can reach full speed
        full_speed_distance = abs(angle) - 2 * distance_accel
        time_at_full_speed = full_speed_distance / speed_deg
        total_time = 2 * time_to_full_speed + time_at_full_speed
    else:
        # If we can't reach full speed
        total_time = 2 * math.sqrt(abs(angle) / adjusted_accel)
    
    # Add a small buffer for real-world factors
    return total_time * 1.1

# Joint parameters
joints = {
    'J1': {'speed': 6000, 'accel': 8000, 'gear_ratio': 6.4},
    'J2': {'speed': 12000, 'accel': 8000, 'gear_ratio': 20.0},
    'J3': {'speed': 14000, 'accel': 8000, 'gear_ratio': 18.0952381},
    'J4': {'speed': 8000, 'accel': 8000, 'gear_ratio': 4.0},
    'J5': {'speed': 10000, 'accel': 8000, 'gear_ratio': 4.0},
    'J6': {'speed': 12000, 'accel': 8000, 'gear_ratio': 10.0}
}

# Get user input for each joint and calculate the longest time
longest_time = 0
for joint in joints:
    while True:
        try:
            angle = float(input(f"Enter the target angle for {joint} (in degrees): "))
            break
        except ValueError:
            print("Invalid input. Please enter a number.")
    
    time = calculate_time(angle, joints[joint]['speed'], joints[joint]['accel'], joints[joint]['gear_ratio'], joint)
    longest_time = max(longest_time, time)

# Add 0.5 seconds to the longest time and print the result
final_time = longest_time + 0.5
print(f"{final_time:.2f}")