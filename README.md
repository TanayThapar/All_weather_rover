# 🚜 All Weather Rover (`all_weather_explorer`)

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Sim-orange?logo=gazebo)
![License](https://img.shields.io/badge/License-Apache%202.0-green)
[![Vercel](https://img.shields.io/badge/Vercel-Live%20Simulator-black?logo=vercel)](https://simulator-bay-gamma.vercel.app)

An autonomous ROS 2 robot platform and simulation workspace designed for navigation, localization, and control in harsh, degraded environmental conditions—such as dense fog, urban canyons, atmospheric dust, and GPS-denied environments.

🌐 **Live 3D Web Simulator:** [https://simulator-bay-gamma.vercel.app](https://simulator-bay-gamma.vercel.app)  
📄 **Academic Research Blueprint:** [docs/RESEARCH_PAPER_GUIDE.md](docs/RESEARCH_PAPER_GUIDE.md)  
🏎️ **Personal Data Training Guide:** [docs/PERSONAL_DATA_TRAINING_GUIDE.md](docs/PERSONAL_DATA_TRAINING_GUIDE.md)

---

## 🔬 Research Focus & Key Breakthrough: Radar-Centric Multi-Sensor Fusion

> [!IMPORTANT]
> **Core Academic Contribution:** Standard visual and optical perception pipelines (e.g., LiDAR SLAM, Visual Odometry) experience severe degradation or total failure in hostile ambient conditions such as dense fog, dust storms, heavy rain, or zero-illumination environments. 
>
> This research project addresses this fundamental vulnerability by placing **4D mmWave Radar at the core of the multi-sensor fusion pipeline**:
> 
> 1. **Weather-Resistant Sensing:** 4D mmWave Radar operates at radio frequencies capable of penetrating dense aerosol particles and fog that completely blind optical LiDAR lasers and RGB/depth cameras.
> 2. **Direct Doppler Velocity Estimation:** Unlike position-derivative odometry from LiDAR/cameras, 4D Radar directly measures instant relative Doppler velocity profiles per point cloud target, providing reliable instantaneous velocity priors during severe dynamic maneuvers.
> 3. **Robust State Estimation & SLAM:** By fusing 4D mmWave Radar target point clouds with IMU inertial integration, Thermal IR vision, and fall-back optical LiDAR, the state estimator maintains resilient state estimation, mapping, and obstacle avoidance even under complete optical blackout ($0\text{ m}$ visual range).

---

## 🌐 Web-Based 3D Physics Simulator & SLAM Testbed

A high-performance web-based 3D robotics simulator built on **Three.js** and **Cannon-es** physics, designed as an accessible Gazebo alternative and digital twin:
👉 **[Launch Live Simulator](https://simulator-bay-gamma.vercel.app)**

- **Realistic Sensor Emulation:** 360° LiDAR, 4D mmWave Radar (77 GHz with Doppler tracking), LWIR Thermal Infrared Camera, RTK GPS, and 6-DOF IMU.
- **Dynamic Weather System:** Dense Fog, Dust Storm, Heavy Rain, Zero-Light Night, and Clear conditions with real-time sensor degradation (beam dropout, particulate scattering, traction reduction).
- **Frontier SLAM & Autonomy:** Live 2D global occupancy grid mapping with autonomous frontier exploration, artificial potential field (APF) obstacle avoidance, landmark recording, and map export.
- **Interactive Controls:** Multiple camera perspectives (Orbit, Chase, FPV, Top-down), live telemetry dashboard, and manual / autonomous drive modes.

### Local Development
```bash
cd simulator
npm install
npm run dev
```
Open `http://localhost:5173` to run the simulator locally.

---

## 🌟 Sensor Payload Overview

The **All Weather Rover** features a modular Xacro component system with Ackermann steering capabilities and multi-sensor fusion:

- **4D mmWave Radar (`radar_front_link`):** *[Primary Sensor for Research]* Front-mounted to penetrate thick fog, dust, and rain while tracking Doppler velocity vector fields and long-range obstacle boundaries.
- **360° LiDAR (`lidar_top_link`):** Mast-mounted high-resolution LiDAR for geometric 3D point-cloud mapping under clear-to-moderate atmospheric conditions.
- **Thermal Infrared Camera (`camera_thermal_link`):** Provides high-contrast thermal vision in zero-light, nocturnal, or smoke-obscured environments.
- **RTK GNSS / GPS (`gps_link`):** Rear-mast mounted positioning module for global reference initialization in open-sky regions.
- **6-DOF IMU (`imu_link`):** Centrally mounted at the exact center-of-mass (CoM) to minimize lever-arm noise during high-rate inertial propagation.
- **Ackermann Steering & Controllers:** Integrated with `ros2_control` and `gz_ros2_control` for realistic vehicle physics and motion modeling.

---

## 📁 Repository Structure

```
All_weather_rover/
├── LICENSE                            # Apache 2.0 license file
├── README.md                          # Project documentation and guide
├── docs/                              # Research documentation and training guides
│   ├── RESEARCH_PAPER_GUIDE.md        # Academic manuscript formulation & protocol
│   └── PERSONAL_DATA_TRAINING_GUIDE.md# Personal data recording, training, and deployment guide
├── simulator/                         # Web-based 3D robotics simulator (Three.js + Cannon-es, deployed on Vercel)
│   ├── index.html                     # Simulation canvas & telemetry HUD
│   ├── package.json                   # Vite & frontend dependencies
│   └── src/
│       ├── main.js                    # 3D physics, SLAM, sensor simulation, and vehicle controller
│       └── style.css                  # HUD and UI styling
└── src/
    └── rover/                         # ROS 2 rover package
        ├── config/
        │   ├── controllers.yaml       # ros2_control configuration (Ackermann & joint state)
        │   └── fusion_params.yaml     # Parameters for DA-EKF and DA-Net fusion node
        ├── launch/
        │   ├── gazebo.launch.py       # Main launch file for Gazebo simulation
        │   ├── spawn.launch.py        # Robot spawner script
        │   └── fusion.launch.py       # Launch file for state estimation fusion node
        ├── rover/                     # Core Python modules
        │   ├── core/                  # ES-EKF, 4D Doppler kinematics, state kinematics
        │   ├── learning/              # DA-Net PyTorch model, dataset loader, multi-task loss
        │   ├── nodes/                 # ROS 2 fusion node & synchronous data recorder
        │   └── scripts/               # Training (train.py), benchmark (evaluate.py), data prep
        ├── urdf/                      # Modular Xacro robot description files
        │   ├── base.xacro             # Main chassis definitions
        │   ├── gazebo.xacro           # Gazebo sensor plugins & physics properties
        │   ├── materials.xacro        # Color & visual materials
        │   ├── ros2_control.xacro     # Hardware interface tags for ros2_control
        │   ├── rover.urdf.xacro       # Top-level Xacro entry point
        │   ├── sensors.xacro          # Sensor frames (Radar, LiDAR, Thermal, IMU, GPS)
        │   └── wheels.xacro           # Wheel geometry & Ackermann steering joints
        ├── worlds/
        │   └── rover_world.sdf        # Gazebo environment (obstacles, lighting & fog)
        ├── package.xml                # ROS 2 package manifest
        ├── setup.py                   # Python package setup configuration
        └── setup.cfg                  # Python packaging configuration
```

---

## 🔬 Research Core: Degradation-Aware 4D Radar-Inertial Fusion (DA-EKF)

The algorithmic novelty implemented in `src/rover/rover/` combines analytical radar kinematics with learned uncertainty estimation:

```
┌─────────────────┐       ┌────────────────────────┐
│ Multi-Modal     │──────>│  DA-Net Neural Model   │
│ Telemetry Feed  │       │ (Trained on User Data) │
└─────────────────┘       └───────────┬────────────┘
                                      │ Dynamic Covariance Scales α(t)
┌─────────────────┐                   │ Body Velocity Priors v_pred
│ 4D mmWave Radar │──┐                ▼
│ Doppler Point-  │  │    ┌────────────────────────┐       ┌──────────────────────┐
│ Cloud Stream    │  ├───>│ Degradation-Aware EKF  │──────>│ Continuous Filtered  │
└─────────────────┘  │    │  (Analytical Doppler   │       │ Odometry & TF        │
┌─────────────────┐  │    │   Jacobians + Gating)  │       │ (/rover/filtered_odom│
│ IMU / Odom/LiDAR│──┘    └────────────────────────┘       └──────────────────────┘
└─────────────────┘
```

### 1. Training on Personal Live Recordings
You can train the model on your personal rover data (NPZ archives, CSV logs, or rosbags):
```bash
python3 -m rover.scripts.train --data_dir data/my_recordings --epochs 40
```
*Supports Apple Silicon Metal GPU acceleration (`device='mps'`), NVIDIA CUDA, and CPU.*

### 2. Evaluating Benchmarks & Baseline Comparisons
Generate publication-quality metrics (ATE, RPE drift) comparing against fixed-covariance baselines:
```bash
python3 -m rover.scripts.evaluate --data data/test_session.npz --checkpoint checkpoints/best_model.pt --save_plot docs/trajectory_benchmark.png
```

### 3. Real-Time ROS 2 Live Deployment
```bash
ros2 launch rover fusion.launch.py \
    checkpoint_path:=$(pwd)/checkpoints/best_model.pt \
    norm_params_path:=$(pwd)/checkpoints/norm_params.json
```

---

## 🤖 Robot Specifications & Sensor Payload

| Component | Joint / Frame Name | Relative Offset `(x, y, z)` | Research Function / Sensor Role |
| :--- | :--- | :--- | :--- |
| **Base Chassis** | `base_link` | `(0.0, 0.0, 0.0)` | Primary reference frame ($80\text{ cm} \times 50\text{ cm} \times 30\text{ cm}$) |
| **IMU** | `imu_link` | `(0.0, 0.0, 0.0)` | Centrally aligned for zero lever-arm noise in EKF/Factor Graph fusion |
| **4D Radar** | `radar_front_link` | `(0.40, 0.0, 0.15)` | **Core Research Sensor**: Fog penetration & instant Doppler velocity measurement |
| **LiDAR** | `lidar_top_link` | `(0.0, 0.0, 0.60)` | Elevated mast mounting for 360° optical point-cloud verification |
| **Thermal Camera** | `camera_thermal_link` | `(0.10, 0.0, 0.50)` | Optical backup providing thermal intensity imagery in zero-light scenarios |
| **RTK GPS** | `gps_link` | `(-0.30, 0.0, 0.80)` | High rear mast position for global ground-truth comparison |

---

## 🚀 Getting Started (ROS 2 / Gazebo)

### Prerequisites

- **ROS 2** (Humble or newer recommended)
- **Gazebo Sim** (`ros_gz_sim`, `ros_gz_bridge`)
- **ros2_control** (`gz_ros2_control`, `ackermann_steering_controller`)

Install dependencies using `rosdep`:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build & Setup

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/TanayThapar/All_weather_rover.git
   ```

2. **Build the workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select rover
   source install/setup.bash
   ```

### Running the Simulation

Launch the rover in the Gazebo environment with full controller integration:
```bash
ros2 launch rover gazebo.launch.py
```

---

## 📜 License

This project is licensed under the [Apache-2.0 License](LICENSE).
