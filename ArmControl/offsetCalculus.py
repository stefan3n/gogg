import numpy as np
from scipy.optimize import minimize

# Constants
L = 49.5  # Segment length in cm

# Manual measurements
measurements = [
    {'angles': (75, 70, 90), 'position': (45, 16)},
    {'angles': (75, 97, 90), 'position': (63, 43)},
    {'angles': (75, 97, 154), 'position': (87, 75)},
    {'angles': (135, 96, 154), 'position': (103, 5)},
    {'angles': (75, 70, 154), 'position': (81, 36)},
]

# Forward kinematics function with offsets
def compute_position(theta1, theta2, theta3, L, a, b, c):
    phi1 = np.radians(theta1 - a)
    phi2 = phi1 + np.radians(theta2 - b)
    phi3 = phi2 + np.radians(theta3 - c)
    
    x = L * np.sin(phi1) + L * np.sin(phi2) + L * np.sin(phi3)
    z = L * np.cos(phi1) + L * np.cos(phi2) + L * np.cos(phi3)
    
    return x, z

# Cost function: sum of squared errors
def cost_function(offsets):
    a, b, c = offsets
    total_error = 0
    for meas in measurements:
        theta1, theta2, theta3 = meas['angles']
        x_true, z_true = meas['position']
        x_pred, z_pred = compute_position(theta1, theta2, theta3, L, a, b, c)
        total_error += (x_pred - x_true)**2 + (z_pred - z_true)**2
    return total_error

# Initial guess for offsets
initial_offsets = [90, 90, 90]  # Start with offsets that align with description

# Optimize
result = minimize(cost_function, initial_offsets, method='Nelder-Mead')
optimal_offsets = result.x
a_opt, b_opt, c_opt = optimal_offsets

print(f"Optimal offsets: a = {a_opt:.2f}, b = {b_opt:.2f}, c = {c_opt:.2f}")