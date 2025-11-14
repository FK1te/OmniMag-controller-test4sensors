# Magnet Controller Utilities

This repository contains scripts for tuning and evaluating the performance of a magnetic control system using both open-loop and closed-loop controllers. It includes tools for trajectory tracking, step response analysis, external magnetic sensing, and parameter optimization.

## Table of Contents

- [Controller Tuning Overview](#controller-tuning-overview)
- [Closed-Loop Gain Tuning](#closed-loop-gain-tuning)
- [Grid Search](#grid-search)
- [Step Response Logging](#step-response-logging)
- [Trajectory Following](#trajectory-following)
- [External Sensor Comparison](#external-sensor-comparison)
- [Robot Motion Data Logging](#robot-motion-data-logging)
- [Notes](#notes)

---

## Controller Tuning Overview

This repository supports a two-phase tuning workflow:

1. **Closed-loop gain tuning** using a GUI
2. **Grid search** to optimize saturation and feedforward parameters

Open-loop control can be tested as a baseline or fallback.

---

## Closed-Loop Gain Tuning

To tune the closed-loop (formerly “PID”) gains interactively:

```bash
python scripts/magnet_controller_closed_loop_gui.py
```

This is typically the first step in tuning. Once initial gains are chosen, proceed to grid search.

---

## Grid Search

To sweep saturation thresholds and feedforward terms for fine‑tuning:

```bash
python tests/grid_search.py
```

This script evaluates controller performance using metrics such as:

- IAE — Integral Absolute Error  
- ISE — Integral Squared Error  
- ITAE — Integral Time Absolute Error  
- ITSE — Integral Time Squared Error  

The grid search is typically the **second tuning stage**, following manual closed-loop parameter initialization.

---

## Step Response Logging

To collect magnetic field responses to step changes in target direction:

```bash
python tests/step_response.py
```

This script measures the system response for different angular steps around multiple axes.  
The data is useful for:

- Validating closed-loop control behavior  
- Comparing open‑loop and closed‑loop responses  

---

## Trajectory Following

To follow a prerecorded magnetic trajectory and log tracking error:

```bash
python follow_trajectory.py
```

This script:

- Interpolates a reference trajectory
- Updates the target vector at the configured sampling rate
- Logs tracking error, measured magnetic field, and currents
- Supports both open-loop and closed-loop controllers

Multiple speeds can be tested to evaluate dynamic performance.

---

## External Sensor Comparison

To use both the robot‑mounted sensor and an external magnetic sensor for cross‑validation:

```bash
python tests/external_measurements.py
```

This script logs:

- Internal magnetic field measurements  
- External sensor measurements  
- Angular tracking error between the two  

Useful for:

- Sensor offset calibration  
- Model validation  
- Quantifying measurement uncertainty  

---

## Robot Motion Data Logging

Collect magnetic field data while the robot executes preconfigured motions.

### On Ubuntu:

```bash
python robot_motion/ubuntu.py
```

### On Windows:

```bash
python robot_motion/windows.py
```

These scripts synchronize robot movement with magnetic field sampling and store complete logs for analysis.

---

## Notes

- Ensure robot, coils, and sensors are powered and connected before starting any script.
- Avoid magnetic disturbances in the environment during experiments.
- All logs are stored automatically and can be used for offline processing, plotting, and benchmarking.