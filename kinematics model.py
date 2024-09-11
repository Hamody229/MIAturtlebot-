import numpy as np

# Robot dimensions
L = 0.3  # Length of the robot (meters)
W = 0.3  # Width of the robot (meters)
r = 0.04  # Radius of the wheels (meters)

# Initial global position and orientation
global_x = 0.0
global_y = 0.0
global_theta = 0.0  # Orientation in radians

def inverse_kinematics(robot_velocities, dt):
    global global_x, global_y, global_theta
    velocities = np.array(robot_velocities).reshape((3, 1))
    
    # Kinematic matrix reflecting distinct wheel contributions
    kinematic_matrix = (1 / r) * np.array([
        [1, -1, -(L / 2 + W / 2)],  # Wheel 1: Front-Left
        [1,  1, -(L / 2 + W / 2)],  # Wheel 2: Front-Right
        [1,  1,  (L / 2 + W / 2)],  # Wheel 3: Rear-Right
        [1, -1,  (L / 2 + W / 2)]   # Wheel 4: Rear-Left
    ])
    
    wheel_velocities = np.dot(kinematic_matrix, velocities)
    
    vx, vy, omega = velocities.flatten()
    
    global_x += (vx * np.cos(global_theta) - vy * np.sin(global_theta)) * dt
    global_y += (vx * np.sin(global_theta) + vy * np.cos(global_theta)) * dt
    global_theta += omega * dt
    
    global_theta = np.arctan2(np.sin(global_theta), np.cos(global_theta))
    
    return wheel_velocities.flatten(), (global_x, global_y, global_theta)

def run_simulation(movement_type, dt, sim_time):
    movements = {
        "forward":  [1, 0, 0],
        "backward": [-1, 0, 0],
        "right":    [0, -1, 0],
        "left":     [0, 1, 0],
        "rotate":   [0, 0, 1],
    }
    
    desired_robot_vels = movements[movement_type]
    
    x_positions, y_positions, theta_positions = [], [], []
    wheel_vels_front_left, wheel_vels_front_right, wheel_vels_rear_right, wheel_vels_rear_left = [], [], [], []
    
    num_steps = int(sim_time / dt)
    
    for _ in range(num_steps):
        wheel_vels, global_position = inverse_kinematics(desired_robot_vels, dt)
        
        wheel_vels_front_left.append(wheel_vels[0])
        wheel_vels_front_right.append(wheel_vels[1])
        wheel_vels_rear_right.append(wheel_vels[2])
        wheel_vels_rear_left.append(wheel_vels[3])
        
        x_positions.append(global_position[0])
        y_positions.append(global_position[1])
        theta_positions.append(global_position[2])
    
    return x_positions, y_positions, theta_positions, wheel_vels_front_left, wheel_vels_front_right, wheel_vels_rear_right, wheel_vels_rear_left

# Parameters
dt = 0.1  # Time step in seconds
sim_time = 5  # Total simulation time in seconds
current_movement = "right"  # Movement type

# Run the simulation
x_positions, y_positions, theta_positions, wheel_vels_front_left, wheel_vels_front_right, wheel_vels_rear_right, wheel_vels_rear_left = run_simulation(current_movement, dt, sim_time)

# Save results to variables for plotting in another cell
results = {
    "x_positions": x_positions,
    "y_positions": y_positions,
    "theta_positions": theta_positions,
    "wheel_vels_front_left": wheel_vels_front_left,
    "wheel_vels_front_right": wheel_vels_front_right,
    "wheel_vels_rear_right": wheel_vels_rear_right,
    "wheel_vels_rear_left": wheel_vels_rear_left
}

# Print results
print(f"Current Movement: {current_movement.capitalize()}")
print("Final Wheel Velocities (Front-Left, Front-Right, Rear-Right, Rear-Left):", wheel_vels_front_left[-1], wheel_vels_front_right[-1], wheel_vels_rear_right[-1], wheel_vels_rear_left[-1])
print("Final Global Position (x, y, theta):", x_positions[-1], y_positions[-1], theta_positions[-1])
