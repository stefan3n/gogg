import numpy as np
import matplotlib.pyplot as plt

# Segment length
L = 49.5

# Angles intervals (degrees)
theta1_range = np.arange(75, 136, 1)   
theta2_range = np.arange(70, 97, 1)   
theta3_range = np.arange(90, 154, 1)  

# Target position
target_x, target_z = 103.0, 5.0

def forward_kinematics(theta1, theta2, theta3, L=49.5, a=3.44, b=83.83, c=-98.83):
    phi1 = np.radians(theta1 - a)
    phi2 = phi1 + np.radians(theta2 - b)
    phi3 = phi2 + np.radians(theta3 - c)
    
    x = L * np.sin(phi1) + L * np.sin(phi2) + L * np.sin(phi3)
    z = L * np.cos(phi1) + L * np.cos(phi2) + L * np.cos(phi3)
    
    return x, z

# Test with a measurement
x, z = forward_kinematics(75, 70, 90)
print(f"Predicted position for (75, 70, 90): x = {x:.2f}, z = {z:.2f}")

# List for storing the end-effector positions.
end_effector_positions = []
best_angles = None
min_error = float('inf')
best_position = None

# Looking for the combination of angles that minimizes the error.
for theta1 in theta1_range:
    for theta2 in theta2_range:
        for theta3 in theta3_range:
            x, z = forward_kinematics(theta1, theta2, theta3, L)
            end_effector_positions.append((x, z))
            # Calculate the error (Euclidean distance)
            error = np.sqrt((x - target_x)**2 + (z - target_z)**2)
            if error < min_error:
                min_error = error
                best_angles = (theta1, theta2, theta3)
                best_position = (x, z)

# Convert to an array for plotting
end_effector_positions = np.array(end_effector_positions)

print(f"Target position: ({target_x}, {target_z}) cm")
print(f"Best angles: θ1 = {best_angles[0]}°, θ2 = {best_angles[1]}°, θ3 = {best_angles[2]}°")
print(f"Obtained position: ({best_position[0]:.2f}, {best_position[1]:.2f}) cm")
print(f"Error: {min_error:.2f} cm")

plt.figure(figsize=(10, 8))
plt.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], s=1, alpha=0.5, label='Workspace')
plt.scatter(target_x, target_z, color='red', s=100, label='Target position', marker='x')
plt.scatter(best_position[0], best_position[1], color='green', s=100, label='Obtained position', marker='o')
plt.xlabel('X (cm)')
plt.ylabel('Z (cm)')
plt.title('The workspace of the end-effector with the target position')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()