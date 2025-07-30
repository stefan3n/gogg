import numpy as np
import matplotlib.pyplot as plt

# Lungimile segmentelor (în cm)
L1 = 49.5
L2 = 49.5
L3 = 49.5

def corrected_forward_kinematics(theta1_deg, theta2_deg, theta3_deg):
    """
    Calculează pozițiile fiecărui joint și coordonatele end-effectorului.
    θ₁: unghi față de bază (orizontală)
    θ₂: unghi între segment 1 și 2
    θ₃: unghi între segment 2 și 3
    Returnează: listă cu puncte [(x0,z0), (x1,z1), (x2,z2), (x3,z3)] și poziția end-effector (x3, z3)
    """
    # Conversie în radiani
    theta1 = np.radians(theta1_deg)
    theta2 = np.radians(theta2_deg)
    theta3 = np.radians(theta3_deg)

    # Calculul orientărilor absolute
    joint1_angle = theta1
    joint2_angle = joint1_angle + (np.pi - theta2)
    joint3_angle = joint2_angle + (np.pi - theta3)

    # Poziții
    x0, z0 = 0, 0
    x1 = x0 + L1 * np.cos(joint1_angle)
    z1 = z0 + L1 * np.sin(joint1_angle)
    
    x2 = x1 + L2 * np.cos(joint2_angle)
    z2 = z1 + L2 * np.sin(joint2_angle)
    
    x3 = x2 + L3 * np.cos(joint3_angle)
    z3 = z2 + L3 * np.sin(joint3_angle)
    
    return [(x0, z0), (x1, z1), (x2, z2), (x3, z3)], (x3, z3)

# Exemplu de unghiuri
theta1, theta2, theta3 = 76, 74, 97
points, end_effector = corrected_forward_kinematics(theta1, theta2, theta3)
x_vals, z_vals = zip(*points)

# Plot
plt.figure(figsize=(8, 6))
plt.plot(x_vals, z_vals, '-o', linewidth=3, markersize=8, color='blue', label="Braț Robotic")
plt.axhline(0, color='gray', linestyle='--', linewidth=1)
plt.axvline(0, color='gray', linestyle='--', linewidth=1)
plt.title("Braț Robotic - Cinematică Directă Corectată")
plt.xlabel("X (cm)")
plt.ylabel("Z (cm)")
plt.grid(True)
plt.axis('equal')
plt.legend()

# Afișare coordonate end-effector
end_x, end_z = end_effector
plt.text(end_x, end_z + 2, f"End-effector:\n({end_x:.1f}, {end_z:.1f}) cm",
         fontsize=10, ha='center', va='bottom', color='darkred')

plt.tight_layout()
plt.show()