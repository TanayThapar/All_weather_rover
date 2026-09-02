# 🚜 All Weather Rover (`all_weather_explorer`)

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Sim-orange?logo=gazebo)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

An autonomous ROS 2 robot platform and simulation workspace designed for navigation, localization, and control in harsh, degraded environmental conditions—such as dense fog, urban canyons, atmospheric dust, and GPS-denied environments.

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
├── src/
│   └── rover/
│       ├── config/
│       │   └── controllers.yaml       # ros2_control configuration (Ackermann & joint state)
│       ├── launch/
│       │   ├── gazebo.launch.py       # Main launch file for Gazebo simulation
│       │   └── spawn.launch.py        # Robot spawner script
│       ├── urdf/                      # Modular Xacro robot description files
│       │   ├── base.xacro             # Main chassis definitions
│       │   ├── gazebo.xacro           # Gazebo sensor plugins & physics properties
│       │   ├── materials.xacro        # Color & visual materials
│       │   ├── ros2_control.xacro     # Hardware interface tags for ros2_control
│       │   ├── rover.urdf.xacro       # Top-level Xacro entry point
│       │   ├── sensors.xacro          # Sensor frames (Radar, LiDAR, Thermal, IMU, GPS)
│       │   └── wheels.xacro           # Wheel geometry & Ackermann steering joints
│       └── worlds/
│           └── rover_world.sdf        # Gazebo environment (obstacles, lighting & fog)
└── package.xml                        # ROS 2 package manifest
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

## 🚀 Getting Started

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

This project is licensed under the [Apache-2.0 License](package.xml).

---

## 🌐 Web-Based 3D Physics Simulator & SLAM Testbed

A web-based 3D robotics simulator built on **Three.js** and **Cannon-es** physics for high-performance simulation on Apple Silicon and web browsers:

- **Realistic Sensor Emulation:** 360° LiDAR, 4D mmWave Radar (77 GHz with Doppler tracking), LWIR Thermal Infrared Camera, RTK GPS, and 6-DOF IMU.
- **Dynamic Weather System:** Dense Fog, Dust Storm, Heavy Rain, Zero-Light Night, and Clear conditions with real-time sensor degradation (beam dropout, particulate scattering, traction reduction).
- **Frontier SLAM & Autonomy:** Live 2D global occupancy grid mapping with autonomous frontier exploration, artificial potential field (APF) obstacle avoidance, landmark recording, and map export.

### Quick Start
```bash
cd simulator
npm install
npm run dev
```
Open `http://localhost:5173` to run the simulator.
