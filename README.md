🚁 C++ Drone Physics Simulator
Overview

This project is a C++ drone physics simulator built from scratch using:

Custom rigid-body physics engine
Euler integration
PID-based attitude control
Telemetry logging to CSV
CMake build system
MSVC toolchain

This version (v1.0) runs as a console-based physics simulation and outputs real-time telemetry data for analysis.
_________________________________________________________________________________________________________________

✨ Features

3D position and velocity simulation
Roll, pitch, yaw orientation tracking
Angular velocity and torque modeling
Basic gravity and thrust modeling
Ground collision handling
Telemetry export (telemetry.csv)
Real-time console output

_________________________________________________________________________________________________________________

🧠 Physics Model

The drone is modeled using:

Newton’s Second Law for linear motion
Torque / Inertia for angular acceleration
Semi-implicit Euler integration

Gravity:

g = -9.81 m/s²

_________________________________________________________________________________________________________________

📊 Output

The simulator generates:

Console output showing time, altitude, velocity, roll, and pitch
A telemetry.csv file for plotting and analysis

Example output:

t=2.0s, z=1.24m, vz=0.82m/s, roll=0.01rad, pitch=0.00rad

_________________________________________________________________________________________________________________

🛠️ Build Instructions (Windows + MSVC)
Requirements

Visual Studio 2026
CMake 3.21+
MSVC compiler

Build

Open project in Visual Studio
Select x64-Debug
Build → Build All
Run project

_________________________________________________________________________________________________________________

📈 Plotting Telemetry (Optional)

To visualize results:

1. Install Python
2. Install matplotlib:

pip install matplotlib

3. Run plotting script:

python plot_telemetry.py

This generates:

Drone altitude vs time
Vertical velocity vs time
Roll & pitch vs time

_________________________________________________________________________________________________________________

🔄 Future Upgrades

Real-time SDL rendering
Keyboard flight control
3D visualization
Autopilot mode
Improved aerodynamic modeling

_________________________________________________________________________________________________________________

👤 Author

Jayson Albizures
C++ / Software Engineering Portfolio Project