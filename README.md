# Autokinesis Tracker Hand Tracking Module | aktrack-ros

This repository contains the hand tracking module for the Autokinesis Tracker system, implemented as a collection of ROS (Robot Operating System) packages for tracking user hand movements and position.

![demo_5min_data](https://github.com/bingogome/aktrack-slicer/blob/main/demo_5min_data.gif)

## Overview

The Autokinesis Tracker hand tracking module provides motion tracking capabilities for the Autokinesis Tracker system. It processes data from tracking devices (optical trackers or joysticks), records movement data during experiments, and communicates with the main control interface (aktrack-slicer) to synchronize experimental protocols.

## System Architecture

The module is divided into several ROS packages:

1. **aktrack_ros** - Main package and launch files
2. **aktrack_ros_comm** - Communication interfaces for UDP connections 
3. **aktrack_ros_comm_decode** - Message decoding and command handling
4. **aktrack_ros_dispatcher** - Coordinates data flow and experiment control
5. **aktrack_ros_kinematics** - Tracks positions and handles transformations
6. **aktrack_ros_universal_utility** - Common utilities used across packages

## Features

- **Motion Tracking**: Captures real-time position data from tracking systems
  - Support for optical trackers (Polaris) via NDI interface
  - Support for joystick/controller input
- **Coordinate Transformation**: Processes and transforms position data through calibrated reference frames
- **Data Recording**: Records tracking data during experimental trials
  - Automatic CSV file generation with timestamps
  - Organized by subject, timestamp, and trial type
- **Network Communication**: 
  - Bidirectional communication with control interface (aktrack-slicer)
  - High-frequency data streaming for visualization
- **Experiment Integration**:
  - Supports multiple experimental protocols (VPB, VPM, VPC)
  - Experiment-triggered data recording

## Requirements

- **Hardware**:
  - Optical tracking system (Polaris) or joystick controller
  - Computer with network connectivity
  - Network connection to other Autokinesis Tracker components
  
- **Software**:
  - Ubuntu 18.04 or later
  - ROS Melodic or Noetic
  - catkin build tools
  - yaml-cpp
  - boost

## Installation

1. Create a ROS workspace (if you don't already have one):
   ```bash
   mkdir -p ~/ws_aktrack/src
   cd ~/ws_aktrack
   catkin init
   ```

2. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/aktrack-ros.git
   ```

3. Use the provided script to create symbolic links:
   ```bash
   cd aktrack-ros
   chmod +x src_to_ws.bash
   ./src_to_ws.bash
   ```

4. Build the workspace:
   ```bash
   cd ~/ws_aktrack
   catkin build
   ```

5. Source the setup file:
   ```bash
   source devel/setup.bash
   ```

## Usage

### Starting the System

Launch the hand tracking module:

```bash
roslaunch aktrack_ros all.launch
```

This will start all necessary nodes for tracking and communication.

### Tracking Modes

The system supports two tracking modes (configured in launch files):

1. **Optical Tracking Mode** (Polaris):
   ```bash
   <!-- In aktrack_ros_kinematics/launch/kinematics_nodes.launch -->
   <node name="node_t_sticker_trackercent" pkg="aktrack_ros_kinematics" type="node_t_sticker_trackercent" output="screen"/>
   <node name="node_dataac_t_sticker_trackercent" pkg="aktrack_ros_kinematics" type="node_dataac_t_sticker_trackercent" output="screen"/>
   <node name="node_viz_t_sticker_trackercent" pkg="aktrack_ros_kinematics" type="node_viz_t_sticker_trackercent" output="screen"/>
   ```

2. **Joystick Mode**:
   ```bash
   <!-- In aktrack_ros_kinematics/launch/kinematics_nodes.launch -->
   <node name="node_t_joystick" pkg="aktrack_ros_kinematics" type="node_t_joystick" output="screen"/>
   <node name="node_dataac_t_joystick" pkg="aktrack_ros_kinematics" type="node_dataac_t_joystick" output="screen"/>
   <node name="node_viz_t_joystick" pkg="aktrack_ros_kinematics" type="node_viz_t_joystick" output="screen"/>
   ```

### Network Configuration

Communication settings can be configured in `aktrack_ros_comm/config_comm.yaml`:

```yaml
PORT_OUT_AK: 8059  # Port for data out from ROS
PORT_IN_AK: 8057   # Port for data into ROS
PORT_OUT_NNBLC_AK: 8083  # High-frequency data port
IP_OUT_NNBLC_AK: "10.17.101.147"  # Target IP address
IP_OUT_AK: "10.17.101.147"
IP_IN_AK: "0.0.0.0"
```

### Data Recording

Data is automatically recorded during experimental trials. Each trial's data is saved as a CSV file with the format:
```
[timestamp]_[subject]_[trial-type].csv
```

The files are stored in the `aktrack_ros/recordeddata/` directory and contain time and position data:
```
time,x,y,z
0.000000,0.023145,0.012853,0.000000
0.016879,0.023148,0.012856,0.000000
...
```

## Calibration

Calibration data for the tracking system is stored in the `aktrack_ros_kinematics/share/` directory:

- `sticker_panelref.yaml` - Panel reference calibration
- `trackerref_trackercent.yaml` - Tracker center calibration

## Experiment Protocol Integration

The system supports several experimental protocols:

- **VPB** (Visual Perception of Bias) - Stationary dot paradigms
  - VPB-hfixed: Head fixed
  - VPB-hfree: Head freely moving

- **VPM** (Visual Perception of Motion) - Moving dot paradigms
  - Format: VPM-[speed]-[direction]
  - Speeds: 2, 4, 6, 8, 12, 18 deg/sec
  - Directions: L (left), R (right), U (up), D (down)

- **VPC** (Visual Perception Calibration) - Fixed-speed calibration
  - Format: VPC-[direction]

## Developer Information

### ROS Topics

Key topics used for internal communication:

- `/AK/Kinematics/T_joystick` - Joystick position data
- `/AK/Kinematics/T_sticker_trackercent` - Optical tracker data
- `/AK/Kinematics/Flag_trial` - Trial control flags
- `/AK/Kinematics/Flag_viz` - Visualization flags
- `/AK/msg_to_send` - Messages to external system
- `/AK/msg_to_send_hi_f` - High-frequency messages to external system
- `/AK/msg_received` - Messages from external system

### Coordinate System

The system uses a right-handed coordinate system. For visualization, the coordinates are converted as follows:
```cpp
// Convert m to mm and convert to RAS coordinates
std::to_string(-point.x * 1000.0) + "_" + std::to_string(-point.y * 1000.0)
```

## Related Projects

- [aktrack-slicer](https://github.com/bingogome/aktrack-slicer) - Main UI control center
- [aktrack-goggle](https://github.com/bingogome/aktrack-goggle) - Eye tracking module 
- [aktrack-screen](https://github.com/bingogome/aktrack-screen) - Visual stimuli module

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Cite

@article{liu2024autokinesis,
  title={Autokinesis Reveals a Threshold for Perception of Visual Motion},
  author={Liu, Yihao and Tian, Jing and Martin-Gomez, Alejandro and Arshad, Qadeer and Armand, Mehran and Kheradmand, Amir},
  journal={Neuroscience},
  volume={543},
  pages={101--107},
  year={2024},
  publisher={Elsevier}
}
