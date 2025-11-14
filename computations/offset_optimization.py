# Magnetic Sensor Offset Optimization (Experimental)
# This script attempts to estimate positional offsets of two Hall-effect sensors by
# minimizing RMSE of reconstructed magnetic moment magnitudes. Not used.

import numpy as np
import pandas as pd
import os
import glob
from scipy.optimize import minimize

# Physical constant
MU_0 = 4 * np.pi * 1e-7  # Vacuum permeability (H/m)

# === Magnetic Field Tensor Calculation ===
def compute_magnetic_tensor(position_vector):
    """Compute dipole tensor A for a given 3D position vector."""
    norm = np.linalg.norm(position_vector)
    if norm == 0:
        raise ValueError("Zero-length vector passed to magnetic tensor computation.")
    factor = MU_0 / (4 * np.pi * norm**3)
    unit_vec = position_vector / norm
    return factor * (3 * np.outer(unit_vec, unit_vec) - np.eye(3))

# === Transformation Matrix Calculation ===
def compute_s_dagger(p1, p2):
    """
    Build S† = pseudoinverse(S)·R, mapping sensor readings to magnetic dipole estimates.
    """
    rotation = np.array([[0, 0, -1],
                         [0, 1,  0],
                         [1, 0,  0]])

    R6 = np.block([[rotation, np.zeros((3, 3))],
                   [np.zeros((3, 3)), rotation]])

    S = np.vstack((compute_magnetic_tensor(p1),
                   compute_magnetic_tensor(p2)))

    return np.linalg.pinv(S) @ R6

# === Data Loading ===
def load_sensor_data(folder):
    """
    Load all CSV logs from the folder and extract sensor data blocks.
    Returns stacked 6×N numeric measurements, ignoring invalid columns.
    """
    csv_files = glob.glob(os.path.join(folder, "*.csv"))
    all_blocks = []

    for file in csv_files:
        try:
            df = pd.read_csv(file, header=None)
            block = pd.to_numeric(df.iloc[:, 4:10].stack(), errors='coerce') \
                       .unstack() \
                       .to_numpy() \
                       .T
            all_blocks.append(block)
        except Exception as exc:
            print(f"Failed to process {file}: {exc}")

    if not all_blocks:
        raise ValueError("No valid sensor logs found.")

    data = np.hstack(all_blocks)
    return data[:, ~np.isnan(data).any(axis=0)]

# === Objective Function (RMSE of magnetic moment magnitude) ===
def rmse_objective(offset_mm, p1_nominal, p2_nominal, data, target_mag):
    """
    Evaluate reconstruction error (RMSE) given an offset guess (mm).
    """
    p1 = (p1_nominal + offset_mm) * 1e-3  # Convert mm → meter
    p2 = (p2_nominal + offset_mm) * 1e-3

    S_dagger = compute_s_dagger(p1, p2)
    dipole_vectors = S_dagger @ data
    norms = np.linalg.norm(dipole_vectors, axis=0)

    return np.sqrt(np.mean((norms - target_mag) ** 2))

# === Main Optimization Routine ===
def run_optimization():
    # Nominal sensor-to-magnet positions (mm)
    p1_nominal = np.array([-14.0, 13.11, -170.0])
    p2_nominal = np.array([-14.0, -13.11, -170.0])
    target_mag = 198.45  # Expected magnitude from calibration

    print("Loading sensor data...")
    data = load_sensor_data("./parameter_estimation_logs")

    initial_err = rmse_objective(
        np.array([0, 0, 0]),
        p1_nominal,
        p2_nominal,
        data,
        target_mag
    )
    print(f"Initial RMSE: {initial_err:.4f}")

    # Search grid for offset starting points (mm)
    search_values = np.linspace(-10, 10, 9)  # 9 points per axis → 729 runs
    bounds = [(-10, 10)] * 3

    best_result = None
    best_rmse = np.inf

    print("Starting grid search with local optimization...")

    for x in search_values:
        for y in search_values:
            for z in search_values:
                start = np.array([x, y, z])
                result = minimize(
                    lambda off: rmse_objective(off, p1_nominal, p2_nominal, data, target_mag),
                    start,
                    method='L-BFGS-B',
                    bounds=bounds
                )

                if result.success and result.fun < best_rmse:
                    best_rmse = result.fun
                    best_result = result

                print(f"Start {start} -> RMSE: {result.fun:.4f}, Success: {result.success}")

    if best_result:
        print("\n=== Best Result Found ===")
        print(f"Optimal Offset (mm): {best_result.x}")
        print(f"Final RMSE: {best_result.fun:.4f}")
    else:
        print("No successful optimization run found.")

if __name__ == "__main__":
    run_optimization()
