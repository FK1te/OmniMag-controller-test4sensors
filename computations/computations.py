# Magnetic Field & Torque Computations for Calibration and Physical Modeling
# This script includes calculations for maximum magnetic field, transformation matrices (S-dagger),
# cylinder inertia, and torque estimation. Not intended for production use.

import numpy as np
import json

pi = np.pi

# --- Magnetic field computation from permanent magnet (cylindrical shape) ---
mu0 = 1.25663706127e-6  # Vacuum permeability (T·m/A)
p = 170e-3              # Distance from magnet in meters

# Magnet dimensions
D = 60e-3  # Diameter (m)
L = 60e-3  # Length (m)
Br = 1.47  # Remanent flux density (T)

# Magnetic moment of the cylinder: m = Br * V / mu0
volume = pi * (D / 2) ** 2 * L
m_mag = Br * volume / mu0
m_dir = m_mag * np.array([0, 0, 1])
print(f"Magnetic field of the permanent magnet: {m_mag:.4f} T")

# Compute B vector using dipole approximation
p_hat = np.array([0, 0, 1])  # Unit vector from magnet to point
B_dir = (mu0 / (4 * pi)) * (1 / p**3) * (3 * np.outer(p_hat, p_hat) - np.eye(3)) @ m_dir
print("B vector (T):", B_dir)
print(f"Max magnetic field: {np.linalg.norm(B_dir) * 1e3:.2f} mT")

# --- Coordinate transformation matrix between sensor frame (MLX) and TCP frame ---
R_mlx_to_tcp = np.array([
    [0, 0, -1],
    [0, 1,  0],
    [1, 0,  0]
])

# Visual verification of transformation
for mlx_vec in [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]:
    print(f"v_mlx: {mlx_vec} v_tcp: {R_mlx_to_tcp @ mlx_vec}")
print(f"Rotational matrix: \n{R_mlx_to_tcp}")

# --- Compute S-dagger matrix ---
mu0 = 4 * np.pi * 1e-7
offset = np.array([-5.1561027, -3.81405932, -2.05598779]) * 1e-3

p_1 = np.array([-14.0, 13.11, -170.0]) * 1e-3
p_2 = np.array([-14.0, -13.11, -170.0]) * 1e-3

def compute_A(p):
    norm = np.linalg.norm(p)
    if norm == 0:
        raise ValueError("Zero-length vector passed to magnetic tensor computation.")
    factor = mu0 / (4 * pi * norm**3)
    p_hat = p / norm
    return factor * (3 * np.outer(p_hat, p_hat) - np.eye(3))

S = np.vstack((compute_A(p_1 + offset), compute_A(p_2 + offset)))

R_combined = np.zeros((6, 6))
R_combined[0:3, 0:3] = R_mlx_to_tcp
R_combined[3:6, 3:6] = R_mlx_to_tcp
print(f"R: \n{R_combined}")

S_inv = np.linalg.pinv(S) @ R_combined
print(f"S dagger: \n{S_inv}")

with open('./S_inv.json', 'w') as f:
    json.dump({'S_inv': S_inv.tolist()}, f)

# --- Compute inertia matrix of the cylindrical magnet ---
radius = 0.03   # m
h = 0.06        # m
density = 7600  # kg/m³

V = pi * radius**2 * h
M = density * V

print(f"The cylinder volume is: {V:.6f} m³")
print(f"The cylinder mass is: {M:.4f} kg")

I = np.zeros((3, 3))
I[0, 0] = (1/12 * M * h**2) + (1/4 * M * radius**2)
I[1, 1] = I[0, 0]
I[2, 2] = (1/2 * M * radius**2)

print(f"Inertial matrix of cylinder: \n{I}")

# --- Compute torque on the magnet ---
m_hat = np.array([1, 0, 0])
B_hat = np.array([0, 1, 0])
B_mag = 10  # Tesla (hypothetical field magnitude)

sin_theta = np.sqrt(1 - (np.dot(m_hat, B_hat))**2)

torque_magnitude = (Br / mu0) * pi * (D/2)**2 * L * B_mag * sin_theta
print(f"Magnitude of magnetic torque: {torque_magnitude:.6f} N·m")

# Sanity checks
print(f"(Br/μ0) * π * R² * L: {(Br / mu0) * pi * (D/2)**2 * L:.4f}")
print(f"Scaled torque (example factor): {(Br / mu0) * pi * (D/2)**2 * L * mu0 * 80 / (90e-3) * 1e-1:.4f}")
