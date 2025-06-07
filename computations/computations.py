# Compute B Max
import numpy as np 
import json

pi = np.pi

mu0 = 1.25663706127e-6  # Vacuum permeability (T·m/A)
p = 170e-3  # Distance from magnet in meters

# Magnet dimensions
D = 60e-3  # Diameter in meters
L = 60e-3  # Length in meters

Br = 1.47  # Remanent flux density in Tesla

# Compute magnetic moment of the cylinder: m = Br * V / mu0
volume = pi * (D / 2) ** 2 * L
m_mag = Br * volume / mu0
m_dir = m_mag * np.array([0, 0, 1])

print(f"Magnetic field of the permanent magnet: {m_mag} T")

# Unit vector from magnet to point
p_hat = np.array([0, 0, 1])

# Magnetic field from dipole at distance p along z-axis
B_dir = (mu0 / (4 * pi)) * (1 / p**3) * (3 * np.outer(p_hat, p_hat) - np.eye(3)) @ m_dir

print("B vector (T):", B_dir)
print(f"Max magnetic field: {np.linalg.norm(B_dir) * 1e3:.2f} mT")

# Compute S
theta = np.deg2rad(-90)

R_mlx_to_tcp = np.array([
    [0, 0, -1],
    [0, 1, 0],
    [1, 0, 0]
])

mlx_vec = np.array([1, 0, 0])
print(f"v_mlx: {mlx_vec} v_tcp:{R_mlx_to_tcp @ mlx_vec}")
mlx_vec = np.array([0, 1, 0])
print(f"v_mlx: {mlx_vec} v_tcp:{R_mlx_to_tcp @ mlx_vec}")
mlx_vec = np.array([0, 0, 1])
print(f"v_mlx: {mlx_vec} v_tcp:{R_mlx_to_tcp @ mlx_vec}")

print(f"Rotational matrix: \n{R_mlx_to_tcp}")

mu0 = 4 * np.pi * 1e-7

offset = np.array([-5.1561027  -3.81405932 -2.05598779]) * 1e-3

p_1 = np.array([-14.0, 13.11, -170.0]) * 1e-3
p_2 = np.array([-14.0, -13.11, -170.0]) * 1e-3

def compute_A(p):
    norm = np.linalg.norm(p)
    if norm == 0:
        raise ValueError("Zero-length vector passed to magnetic tensor computation.")
    factor = 4 * np.pi * 1e-7 / (4 * np.pi * norm**3)
    p_hat = p / norm
    return factor * (3 * np.outer(p_hat, p_hat) - np.eye(3))

S = np.vstack((compute_A(p_1 + offset), compute_A(p_2 + offset)))

R_mlx_to_tcp_combined = np.zeros((6,6))
R_mlx_to_tcp_combined[0:3, 0:3] = R_mlx_to_tcp.copy()
R_mlx_to_tcp_combined[3:6, 3:6] = R_mlx_to_tcp.copy()

print(f"R: \n{R_mlx_to_tcp_combined}")

S_inv = np.linalg.pinv(S) @ R_mlx_to_tcp_combined

print(f"S dagger: \n{S_inv}")
with open('./S_inv.json', 'w') as f:
    json.dump({'S_inv': S_inv.tolist()}, f)

# Compute I
radius = 0.03   # m
h = 0.06        # m
density = 7600  # kg/m3

V = np.pi * (radius ** 2) * h
M = density * V

print(f"The cylinder volume is: {V:.6f} m³")
print(f"The cylinder mass is: {M:.4f} kg")

I = np.zeros((3,3))
I[0, 0] = (1/12 * M * h**2) + (1/4*M*radius**2)
I[1, 1] = (1/12 * M * h**2) + (1/4*M*radius**2)
I[2, 2] = (1/2*M*radius**2)


print(f"Inertial matrix of cylinder: \n{I}")

# Compute magnitude of the torque exerted on the permanent magnet

m_hat = np.array([1, 0, 0])
B_hat = np.array([0, 1, 0])
B_mag = 10

# Magnet dimensions
D = 60e-3  # Diameter in meters
R_square = (D/2)**2
L = 60e-3  # Length in meters
pi = 3.14159265359
sin_theta = np.sqrt(1 - (np.dot(m_hat, B_hat))**2)
Br = 1.47  # Remanent flux density in Tesla
mu0 = 1.25663706127e-6  # Vacuum permeability (T·m/A)
torque_magnitude = (Br/mu0) * pi * R_square * L * B_mag * sin_theta
print(f"Magnitude of magnetic torque: {torque_magnitude} N.m")
print(f"{(Br/mu0) * pi * R_square * L}")

print(f"{(Br/mu0) * pi * R_square * L * mu0 * 80/(90e-3)*1e-1}")