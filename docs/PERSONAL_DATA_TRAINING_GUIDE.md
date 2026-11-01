# 🏎️ Personal Live Data Training & Recording Guide

This guide walks you step-by-step through recording live data on your physical rover (or simulation), preprocessing the recordings, training the **Degradation-Aware Neural Network (DA-Net)**, evaluating trajectory accuracy, and deploying the model live in ROS 2.

---

## 📋 Table of Contents
1. [Sensor Setup & Coordinate Conventions](#1-sensor-setup--coordinate-conventions)
2. [Recording Live Telemetry](#2-recording-live-telemetry)
   - [Method A: Using `data_recorder_node` (Recommended)](#method-a-using-data_recorder_node-recommended)
   - [Method B: Using Standard ROS 2 Rosbags](#method-b-using-standard-ros-2-rosbags)
   - [Method C: Logging to CSV](#method-c-logging-to-csv)
3. [Preprocessing Data for Training](#3-preprocessing-data-for-training)
4. [Training the Model (`train.py`)](#4-training-the-model-trainpy)
5. [Benchmarking & Evaluating Trajectories (`evaluate.py`)](#5-benchmarking--evaluating-trajectories-evaluatepy)
6. [Live Deployment on the Rover](#6-live-deployment-on-the-rover)

---

## 1. Sensor Setup & Coordinate Conventions

For optimal multi-sensor fusion:
* **IMU (`/imu`):** Mount as close as possible to the robot's Center-of-Mass (CoM) to minimize lever-arm inertial noise.
* **4D mmWave Radar (`/radar/scan`):** Forward-facing, typically mounted on the front bumper ($x \approx +0.40\text{ m}, z \approx +0.15\text{ m}$).
* **Wheel Encoders (`/odom`):** Measures linear forward velocity $v_x$ and yaw rate $\omega_z$.
* **Ground Truth Source:** During baseline training runs, use RTK-GPS (`/gps/fix`), an optical flow ground-truth sensor, or loop-closed LiDAR SLAM in clear conditions to provide the supervision ground truth $\mathbf{v}_{\text{gt}}$.

---

## 2. Recording Live Telemetry

### Method A: Using `data_recorder_node` (Recommended)

The built-in `data_recorder_node` automatically subscribes to all active sensor topics, synchronizes them at 20 Hz, and exports compressed `.npz` files ready for training.

```bash
# In your ROS 2 workspace:
source install/setup.bash

# Start recording a new session:
ros2 run rover data_recorder_node --ros-args -p output_dir:=data/my_recordings -p session_name:=outdoor_run_01
```

* Drive your rover through various terrain and weather conditions.
* Press `Ctrl+C` to cleanly terminate recording. The node automatically writes `data/my_recordings/outdoor_run_01.npz`.

---

### Method B: Using Standard ROS 2 Rosbags

If you prefer recording raw rosbags:

```bash
ros2 bag record -o my_rover_bag /imu /radar/scan /odom /gps/fix
```

Then extract the bag into training arrays using the preprocessor:
```bash
python3 -m rover.scripts.prepare_personal_data --input my_rover_bag --output data/my_recordings/bag_session.npz
```

---

### Method C: Logging to CSV

If your microcontroller or onboard computer logs to CSV, format your columns as follows:

| Column Name | Description | Units |
| :--- | :--- | :--- |
| `timestamp` | Epoch or relative seconds | $\text{s}$ |
| `radar_count` | Number of detected radar targets | Count |
| `radar_dop` | Mean radial Doppler velocity | $\text{m/s}$ |
| `radar_snr` | Average target Signal-to-Noise Ratio | $\text{dB}$ |
| `lidar_points` | Valid LiDAR point cloud returns | Count |
| `lidar_range` | Mean obstacle distance | $\text{m}$ |
| `lidar_backscatter` | Near-field backscatter ratio ($<1.0\text{ m}$) | $[0, 1]$ |
| `acc_mag` | Linear acceleration magnitude | $\text{m/s}^2$ |
| `gyr_mag` | Angular velocity magnitude | $\text{rad/s}$ |
| `wheel_speed` | Measured forward speed | $\text{m/s}$ |
| `gt_vx`, `gt_vy` | Reference ground truth velocity | $\text{m/s}$ |

Convert the CSV to training format:
```bash
python3 -m rover.scripts.prepare_personal_data --input session_raw.csv --output data/my_recordings/session_01.npz
```

---

## 3. Preprocessing Data for Training

Verify that your `.npz` file contains the following arrays:
```python
import numpy as np
data = np.load('data/my_recordings/outdoor_run_01.npz')
print(data.files)
# Expected keys: ['features', 'gt_velocity', 'timestamps']
# features shape: (N, 16)
# gt_velocity shape: (N, 3)
```

---

## 4. Training the Model (`train.py`)

Train the network using Apple Silicon GPU acceleration (Metal / `mps`), NVIDIA CUDA, or CPU:

```bash
# Single session or entire folder of recordings:
python3 -m rover.scripts.train \
    --data_dir data/my_recordings \
    --epochs 40 \
    --batch_size 32 \
    --lr 0.001 \
    --checkpoint_dir checkpoints
```

### Key Training Outputs:
* `checkpoints/best_model.pt`: Saved model weights at lowest validation loss.
* `checkpoints/norm_params.json`: Feature mean and standard deviation scaling vector (essential for real-time live inference).

### Training Hyperparameters:
* `--epochs`: Default 30–40. Early stopping terminates automatically if loss stops improving.
* `--batch_size`: 16 to 64 depending on session length.
* `--lr`: Default `1e-3` with Cosine Annealing learning rate schedule.

---

## 5. Benchmarking & Evaluating Trajectories (`evaluate.py`)

Evaluate the trained model on an independent test recording and compare against standard baselines:

```bash
python3 -m rover.scripts.evaluate \
    --data data/my_recordings/test_run.npz \
    --checkpoint checkpoints/best_model.pt \
    --save_plot docs/trajectory_benchmark.png
```

### Generated Benchmark Metrics:
* **ATE RMSE (m):** Absolute Trajectory Error against ground truth.
* **RPE Drift (m):** Relative pose drift across sliding intervals.
* **Trajectory Figure:** Dual-panel publication-grade plot showing trajectory overlay and dynamic sensor confidence weights ($w_{\text{radar}}, w_{\text{lidar}}, w_{\text{wheel}}$).

---

## 6. Live Deployment on the Rover

Once trained, launch the real-time ROS 2 state estimation node with your saved checkpoint:

```bash
ros2 launch rover fusion.launch.py \
    checkpoint_path:=$(pwd)/checkpoints/best_model.pt \
    norm_params_path:=$(pwd)/checkpoints/norm_params.json
```

The node will:
1. Stream incoming sensor telemetry from `/imu`, `/radar/scan`, and `/odom`.
2. Infer real-time sensor reliability weights and dynamic measurement noise $\mathbf{R}_k(t)$.
3. Execute the Error-State Kalman Filter with Doppler constraints.
4. Publish `/rover/filtered_odom` and broadcast the coordinate transform `odom -> base_link`.
