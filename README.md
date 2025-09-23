# Magnet Controller Utilities

This repository contains various scripts to assist in tuning and evaluating the performance of a closed-loop control system using PID controllers, trajectory tracking, and external sensor integration.

## Table of Contents

- [PID Gain Tuning](#pid-gain-tuning)
- [Grid Search for Magnet Tuning](#grid-search-for-magnet-tuning)
- [Step Response Data Collection](#step-response-data-collection)
- [Trajectory Following](#trajectory-following)
- [External Sensor Measurements](#external-sensor-measurements)
- [Measurements During Robot Motion](#measurements-during-robot-motion)

---

## PID Gain Tuning

To tune the PID gains for closed-loop control, use the GUI-based script:

```bash
python scripts/magnet_controller_pid_gui.py
```

---

## Grid Search for Magnet Tuning

To perform grid search for tuning other magnet control parameters, use:

```bash
python test/grid_search.py
```

---

## Step Response Data Collection

To collect step response data:

```bash
python test/step_response.py
```

---

## Trajectory Following

To capture trajectory following data:

```bash
python follow_trajectory.py
```

---

## External Sensor Measurements

For measurements using an external sensor:

```bash
python test/external_measurements.py
```

---

## Measurements During Robot Motion

For collecting data while the robot is in motion, use the following platform-specific scripts:

### On Ubuntu:

Run the following script from the Ubuntu terminal:

```bash
python robot_motion/ubuntu.py
```

### On Windows:

Run the following script from the Windows terminal:

```bash
python robot_motion/windows.py
```

---

## Notes

- Ensure all dependencies are installed and that the robot is properly connected before running any of the scripts.
- It is recommended to run these scripts in a controlled environment to avoid unexpected behavior during tuning and data collection.
