# 🚜 All Weather Rover (`all_weather_explorer`)

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/Gazebo-Sim-orange?logo=gazebo)
![License](https://img.shields.io/badge/License-Apache%202.0-green)

An autonomous ROS 2 robot platform and simulation workspace designed for navigation, localization, and control in harsh, degraded environmental conditions—such as dense fog, urban canyons, atmospheric dust, and GPS-denied environments.

---

## 🌟 Overview

Standard visual/optical sensors like LiDAR and RGB cameras degrade significantly during adverse weather or dense fog. The **All Weather Rover** features a modular Xacro component system with Ackermann steering capabilities and multi-sensor fusion:

- **4D mmWave Radar (`radar_front_link`):** Front-mounted to penetrate thick fog, dust, and rain while tracking doppler velocity and obstacle range.
- **360° LiDAR (`lidar_top_link`):** Mast-mounted high-resolution LiDAR for 3D point-cloud map building in clear-to-moderate conditions.
- **Thermal Infrared Camera (`camera_thermal_link`):** Provides high-contrast vision in zero-light or low-visibility scenarios.
- **RTK GNSS / GPS (`gps_link`):** Rear-mast mounted positioning module for open-sky localization.
- **6-DOF IMU (`imu_link`):** Centrally mounted at the center-of-mass to minimize lever-arm noise during state estimation.
- **Ackermann Steering & Controllers:** Integrated with `ros2_control` and `gz_ros2_control` for realistic vehicle physics.

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

| Component | Joint / Frame Name | Relative Offset `(x, y, z)` | Function / Description |
| :--- | :--- | :--- | :--- |
| **Base Chassis** | `base_link` | `(0.0, 0.0, 0.0)` | Primary reference frame ($80\text{ cm} \times 50\text{ cm} \times 30\text{ cm}$) |
| **IMU** | `imu_link` | `(0.0, 0.0, 0.0)` | Mounted at CoM for precise inertial tracking |
| **4D Radar** | `radar_front_link` | `(0.40, 0.0, 0.15)` | Low front placement for fog penetration & velocity tracking |
| **LiDAR** | `lidar_top_link` | `(0.0, 0.0, 0.60)` | Elevated mast mounting for unobstructed 360° FOV |
| **Thermal Camera** | `camera_thermal_link` | `(0.10, 0.0, 0.50)` | Forward-facing thermal vision sensor |
| **RTK GPS** | `gps_link` | `(-0.30, 0.0, 0.80)` | High rear mast position for satellite line-of-sight |

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
