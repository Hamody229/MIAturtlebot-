import matplotlib.pyplot as plt

def plot_wheel_velocities_bar(wheel_vels1, wheel_vels2, wheel_vels3, wheel_vels4):
    final_vels = [wheel_vels1[-1], wheel_vels2[-1], wheel_vels3[-1], wheel_vels4[-1]]
    
    # Wheel labels
    wheel_labels = ['front_left', 'front_right', 'rear_right', 'rear_left']
    
    # Create bar graph
    plt.figure(figsize=(8, 6))
    plt.bar(wheel_labels, final_vels, color=['r', 'g', 'b', 'c'])
    plt.xlabel('Wheel')
    plt.ylabel('Velocity (rad/s)')
    plt.title('Final Wheel Velocities After Simulation')
    plt.grid(True, axis='y', linestyle='--', alpha=0.7)
    plt.show()

wheel_vels1 = results["wheel_vels_front_left"]
wheel_vels2 = results["wheel_vels_front_right"]
wheel_vels3 = results["wheel_vels_rear_right"]
wheel_vels4 = results["wheel_vels_rear_left"]

plot_wheel_velocities_bar(wheel_vels1, wheel_vels2, wheel_vels3, wheel_vels4)
