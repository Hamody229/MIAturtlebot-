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
    # Robot velocities: [vx, vy, omega]
    velocities = np.array(robot_velocities).reshape((3, 1))  # Reshape into a 3x1 column vector
    
    # Kinematic matrix for a 4-wheel robot
    kinematic_matrix = (1 / r) * np.array([
        [1, -1, -(L + W)],
        [1,  1,  (L + W)],
        [1, -1,  (L + W)],
        [1,  1, -(L + W)]
    ])
    
    # Calculate wheel velocities: [omega1, omega2, omega3, omega4]
    wheel_velocities = np.dot(kinematic_matrix, velocities)
    
    # Extract the linear and angular velocity components
    vx, vy, omega = velocities.flatten()
    
    # Update global position and orientation based on velocity
    # Global frame translation using robot's orientation (global_theta)
    global_x += (vx * np.cos(global_theta) - vy * np.sin(global_theta)) * dt
    global_y += (vx * np.sin(global_theta) + vy * np.cos(global_theta)) * dt
    global_theta += omega * dt
    
    # Ensure global_theta stays within the range [-pi, pi]
    global_theta = np.arctan2(np.sin(global_theta), np.cos(global_theta))
    
    return wheel_velocities.flatten(), (global_x, global_y, global_theta)

# Time step and simulation duration
dt = 0.1  # Time step in seconds
sim_time = 5  # Total simulation time in seconds

# Define movement types
movements = {
    "forward":  [1, 0, 0],    # Move forward (vx = 1 m/s, vy = 0, omega = 0)
    "backward": [-1, 0, 0],   # Move backward (vx = -1 m/s, vy = 0, omega = 0)
    "right":    [0, -1, 0],   # Move to the right (vy = -1 m/s, vx = 0, omega = 0)
    "left":     [0, 1, 0],    # Move to the left (vy = 1 m/s, vx = 0, omega = 0)
    "rotate":   [0, 0, 1],    # Rotate in place (omega = 1 rad/s, vx = 0, vy = 0)
}

# Choose the desired movement
current_movement = "right"  # Change to "backward", "right", "left", or "rotate" to test other movements
desired_robot_vels = movements[current_movement]

# Lists to store the robot's global position over time
x_positions = []
y_positions = []
theta_positions = []

# Run the simulation for the given time
num_steps = int(sim_time / dt)
for step in range(num_steps):
    wheel_vels, global_position = inverse_kinematics(desired_robot_vels, dt)
    
    # Store positions for plotting
    x_positions.append(global_position[0])
    y_positions.append(global_position[1])
    theta_positions.append(global_position[2])
    
# Print the current movement, final wheel velocities, and global position at the end of the simulation
print(f"Current Movement: {current_movement.capitalize()}")
print("Final Wheel Velocities (omega1, omega2, omega3, omega4):", wheel_vels)
print("Final Global Position (x, y, theta):", global_position)
