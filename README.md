# ENT Robot Calibration, Navigation, and Visualization

## drillViz/drilling.py

https://github.com/user-attachments/assets/fd9796a3-4fb3-46b2-9828-444cb7d0e994

This code uses Python 3.8.10.

Command-line tool to generate and visualize drilling trajectories on anatomical meshes using interactive point selection and spline-based path planning.

### Features

- Interactive 3D mesh visualization with PyVista
- Right-click to select 3 target points (start, midpoint, end) on the mesh surface
- Automatic drilling path generation through surface intersection
- Smooth spline interpolation for trajectory smoothing
- Real-time drill animation with proper orientation (45-degree drilling angle)
- Press 'c' to reset and select new target points
- Visualizes plane intersection, spline path, and target points in 3D

https://github.com/user-attachments/assets/d3ff68a3-32b9-4695-a259-b45feb5d314a

**CalibrateRobotTracker.py** calibrates the marker2gripper homogeneous transformation matrix using OpenCV, extracting the needed data from a .bag file, or by listening to provided topics posting the desired poses.

**VisualizeRobotTracker.py** produces a 3D visualization of the various frames of a surgical robot in the base frame, including the base, optical tracker origin, and the corresponding markers/tip of the tool (endoscope tip here).

This code was tested using the Noetic distro of ROS, also using Python 3.8.10, in WSL2: Ubuntu-20.04. Ensure ROS Noetic is installed and call `roscore` prior to live calibration via the listener node.

## CalibrateRobotTracker

Command-line tool to calibrate a robot and tracker by listening to ROS `PoseStamped` topics or a ROS bag file.

### Basic usage

python3 CalibrateRobotTracker.py [output_path]

- `output_path` (optional): Path to the output `.txt` file where the calibrated marker2gripper matrix will be saved.
- Defaults to `../output/marker2gripper.txt` if omitted.

### Options

- `--custom_topics <hand_topic> <eye_topic>`: Override default topics (`/REMS/Research/measured_cp` for hand, `/atracsys/Endoscope/measured_cp` for eye).
- `--from_bag <bag_path>`: Use data from a ROS bag file instead of live topics.
- `--max_samples <number>`: Set maximum samples to collect (default: 400).

**Example:**

python3 CalibrateRobotTracker.py results/calibration.txt --from_bag data/session1.bag --max_samples 500

## VisualizeRobotTracker

Command-line tool to visualize robot poses and transformations using PyVista 3D rendering and ROS `PoseStamped` topics.

<img width="541" height="322" alt="visualizeRobotFigure_surgicalScale_labeled" src="https://github.com/user-attachments/assets/6f5e0569-1946-44f6-ae92-91403ae5743d" />
(2cm surgical scale axes)

### Features

- Live 3D visualization of gripper (red), endoscope (green), tracker (blue), and anatomy (brown) poses
- Automatically subscribes to ROS topics and updates visualization in real-time
- Static black axes at robot base (0,0,0)

The tool runs ROS listener nodes by default and renders pose transformations using the provided matrices.


