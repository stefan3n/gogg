import numpy as np
import matplotlib.pyplot as plt

# Segment length (poate necesita ajustare bazată pe măsurători reale)
L = 49.5

# Offset-uri unghiulare inițiale (trebuie calibrate cu date reale)
a = 3.44  # offset pentru theta1
b = 83.83  # offset pentru theta2
c = -98.83  # offset pentru theta3

# Intervalele unghiurilor (în grade)
theta1_range = np.arange(75, 136, 1)   
theta2_range = np.arange(70, 97, 1)   
theta3_range = np.arange(90, 154, 1)  

# Poziția țintă (în sistemul de coordonate al modelului, z=0 la bază)
target_x, target_z = 103.0, 5.0

# Offset-ul bazei (înălțimea bazei deasupra solului)
z_offset = 12.0  # cm

def forward_kinematics(theta1, theta2, theta3, L=L, a=a, b=b, c=c):
    """
    Calculează poziția efectorului final pe baza unghiurilor articulațiilor.
    
    Parametri:
    - theta1, theta2, theta3: unghiuri în grade
    - L: lungimea segmentului în cm
    - a, b, c: offset-uri unghiulare în grade
    
    Returnează:
    - x, z: poziția în cm, relativă la bază (z=0 la bază)
    """
    phi1 = np.radians(theta1 - a)
    phi2 = phi1 + np.radians(theta2 - b)
    phi3 = phi2 + np.radians(theta3 - c)
    
    x = L * np.sin(phi1) + L * np.sin(phi2) + L * np.sin(phi3)
    z = L * np.cos(phi1) + L * np.cos(phi2) + L * np.cos(phi3)
    
    return x, z

# Listă pentru stocarea pozițiilor efectorului final
end_effector_positions = []
best_angles = None
min_error = float('inf')
best_position = None

# Căutarea combinației de unghiuri care minimizează eroarea
for theta1 in theta1_range:
    for theta2 in theta2_range:
        for theta3 in theta3_range:
            x, z = forward_kinematics(theta1, theta2, theta3)
            end_effector_positions.append((x, z))
            # Calculează eroarea (distanța euclidiană)
            error = np.sqrt((x - target_x)**2 + (z - target_z)**2)
            if error < min_error:
                min_error = error
                best_angles = (theta1, theta2, theta3)
                best_position = (x, z)

# Conversie la array pentru plotare
end_effector_positions = np.array(end_effector_positions)

# Afișare rezultate
print(f"Poziția țintă: ({target_x}, {target_z}) cm (relativ la bază)")
print(f"Cele mai bune unghiuri: θ1 = {best_angles[0]}°, θ2 = {best_angles[1]}°, θ3 = {best_angles[2]}°")
print(f"Poziția obținută conform modelului: ({best_position[0]:.2f}, {best_position[1]:.2f}) cm (relativ la bază)")
print(f"Poziția așteptată măsurată de la sol: x = {best_position[0]:.2f}, z = {best_position[1] + z_offset:.2f} cm")

# Notă despre calibrare:
# Dacă pozițiile măsurate (de la sol) nu se potrivesc cu cele calculate (ajustate cu z_offset),
# parametrii modelului (a, b, c, L) trebuie calibrați.
# Pași sugerati pentru calibrare:
# 1. Setează brațul la unghiuri cunoscute (ex. 75°, 70°, 90°).
# 2. Măsoară poziția fizică (x_m, z_m) de la sol.
# 3. Calculează z relativ la bază: z_model = z_m - 12.
# 4. Ajustează a, b, c și/sau L astfel încât forward_kinematics(θ1, θ2, θ3) = (x_m, z_model).
# 5. Repetă pentru mai multe seturi de unghiuri și optimizează folosind minimizarea erorii (ex. least squares).

# Plotare
plt.figure(figsize=(10, 8))
plt.scatter(end_effector_positions[:, 0], end_effector_positions[:, 1], s=1, alpha=0.5, label='Spațiul de lucru')
plt.scatter(target_x, target_z, color='red', s=100, label='Poziția țintă', marker='x')
plt.scatter(best_position[0], best_position[1], color='green', s=100, label='Poziția obținută (model)', marker='o')
plt.xlabel('X (cm)')
plt.ylabel('Z (cm, relativ la bază)')
plt.title('Spațiul de lucru al efectorului cu poziția țintă')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()