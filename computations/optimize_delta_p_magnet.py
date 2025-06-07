import numpy as np
import pandas as pd
import os
import glob
from scipy.optimize import minimize


# Physical constant
MU_0 = 4 * np.pi * 1e-7  # Vacuum permeability (H/m)

# === Magnetic Field Tensor Calculation ===
def compute_magnetic_tensor(p):
    norm = np.linalg.norm(p)
    if norm == 0:
        raise ValueError("Zero-length vector passed to magnetic tensor computation.")
    factor = MU_0 / (4 * np.pi * norm**3)
    p_hat = p / norm
    return factor * (3 * np.outer(p_hat, p_hat) - np.eye(3))

# === Transformation Matrix Calculation ===
def compute_s_dagger(p1, p2):
    R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    R6 = np.block([[R, np.zeros((3, 3))], [np.zeros((3, 3)), R]])
    S = np.vstack((compute_magnetic_tensor(p1), compute_magnetic_tensor(p2)))
    return np.linalg.pinv(S) @ R6

# === Data Loading ===
def load_sensor_data(folder):
    csv_files = glob.glob(os.path.join(folder, "*.csv"))
    all_data = []
    for file in csv_files:
        try:
            df = pd.read_csv(file, header=None)
            sensor_block = pd.to_numeric(df.iloc[:, 4:10].stack(), errors='coerce').unstack().to_numpy().T
            all_data.append(sensor_block)
        except Exception as e:
            print(f"Failed to process {file}: {e}")
    if not all_data:
        raise ValueError("No valid data found.")
    data = np.hstack(all_data)
    return data[:, ~np.isnan(data).any(axis=0)]

# === Objective Function ===
def rmse_objective(offset, p1_est, p2_est, data, target_magnitude):
    p1 = (p1_est + offset) * 1e-3  # mm to meters
    p2 = (p2_est + offset) * 1e-3
    S_dagger = compute_s_dagger(p1, p2)
    m_vectors = S_dagger @ data
    norms = np.linalg.norm(m_vectors, axis=0)
    return np.sqrt(np.mean((norms - target_magnitude) ** 2))
def run_optimization():
    # Initial magnet positions (in mm)
    p1_est = np.array([-14.0, 13.11, -170.0])
    p2_est = np.array([-14.0, -13.11, -170.0])
    target_magnitude = 198.45

    print("Loading sensor data...")
    data = load_sensor_data("./parameter_estimation_logs")

    print("Initial error:", rmse_objective(np.array([0, 0, 0]), p1_est, p2_est, data, target_magnitude))

    # === Grid Search Parameters ===
    search_range = np.linspace(-10, 10, 9)  # 9 points per axis → 729 total
    bounds = [(-10, 10)] * 3

    best_result = None
    best_fun = np.inf

    print("Starting grid search with local optimization...")

    for x in search_range:
        for y in search_range:
            for z in search_range:
                start = np.array([x, y, z])
                res = minimize(
                    lambda offset: rmse_objective(offset, p1_est, p2_est, data, target_magnitude),
                    start,
                    method='L-BFGS-B',
                    bounds=bounds
                )
                if res.success and res.fun < best_fun:
                    best_fun = res.fun
                    best_result = res

                print(f"Start {start} -> RMSE: {res.fun:.4f}, Success: {res.success}")

    if best_result:
        print("\nBest result found:")
        print(f"Optimal Offset (mm): {best_result.x}")
        print(f"Final RMSE: {best_result.fun:.4f}")
    else:
        print("No successful optimization run found.")

if __name__ == "__main__":
    run_optimization()
