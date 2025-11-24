# ROS2 3R Robot Arm Control System

A comprehensive ROS2 package for controlling a 3-DOF revolute joint robot arm with multiple operation modes including teleoperation, trajectory planning, and inverse kinematics.

## System Overview

This system implements a complete robotic arm control solution with the following key features:

- **3-DOF Robot Arm**: Modified Denavit-Hartenberg (MDH) parameterized robot with 3 revolute joints
- **Multiple Control Modes**: Inverse kinematics, teleoperation, and autonomous trajectory execution
- **Real-time Visualization**: RViz2 integration with live robot state display
- **Workspace Safety**: Built-in collision detection and workspace boundary enforcement
- **Keyboard Teleoperation**: Intuitive keyboard control with frame switching capabilities

## Architecture

The system consists of 5 main ROS2 nodes that communicate through topics and services:

```
teleop_jog_keyboard → control_node ↔ random_service_node
                           ↓
                    robot_state_publisher → rviz2
```

### Node Descriptions

| Node | Purpose | Key Features |
|------|---------|--------------|
| `control_node` | Main robot controller | IK solver, teleoperation, trajectory execution |
| `random_service_node` | Random pose generator | FK-based valid pose generation |
| `teleop_jog_keyboard` | Keyboard interface | Real-time teleoperation control |
| `robot_state_publisher` | Robot visualization | URDF/Xacro model publishing |
| `rviz2` | 3D visualization | Real-time robot state display |

## Robot Specifications

### Denavit-Hartenberg Parameters

| Joint | α (rad) | a (m) | d (m) | θ (rad) |
|-------|---------|-------|-------|---------|
| 1     | 0.0     | 0.0   | 0.2   | q₁      |
| 2     | π/2     | 0.0   | 0.02  | q₂      |
| 3     | 0.0     | 0.25  | 0.0   | q₃      |

**Tool Transform**: Translation of 0.28m along X-axis  
**Total Reach**: 0.53m (maximum extension)  
**Minimum Reach**: 0.03m (collision avoidance)

### Workspace Constraints

- **Spherical workspace** centered at shoulder joint (0, 0, 0.2)
- **Radial limits**: 0.03m ≤ r ≤ 0.53m
- **Ground clearance**: z ≥ 0.0m
- **Singularity detection** with 0.0001 determinant threshold

## Control Modes

### Mode 1: Inverse Kinematics
- **Input**: Target end-effector position (x, y, z)
- **Method**: Levenberg-Marquardt optimization
- **Features**: Automatic workspace validation, singularity handling

### Mode 2: Teleoperation
- **Input**: Real-time velocity commands from keyboard
- **Frames**: World frame or end-effector frame control
- **Safety**: Real-time workspace boundary checking
- **Jacobian**: Position-only control (3×3 Jacobian matrix)

### Mode 3: Trajectory Execution
- **Generation**: Quintic polynomial joint-space trajectories
- **Duration**: 1.0 second per trajectory segment
- **Points**: 100 interpolated waypoints
- **Targets**: Random valid poses from FK-based generation

## Installation & Dependencies

### System Requirements
- **ROS2 Humble** (Ubuntu 22.04)
- **Python 3.10+**
- **RViz2** for visualization

### Python Dependencies
```bash
pip install roboticstoolbox-python spatialmath-python numpy
pip3 install numpy==1.26.4
pip3 install roboticstoolbox-python
sudo apt install ros-humble-desktop-full
sudo apt install ros-dev-tools
sudo apt install ros-humble-teleop-twist-keyboard
```

### ROS2 Dependencies
```bash
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
```

### Custom Interfaces
The system requires custom service definitions:
```bash
# robot_interfaces/srv/Mode.srv
int32 data
geometry_msgs/Point position
---
bool success

# robot_interfaces/srv/Random.srv
int32 data
---
geometry_msgs/Point position
bool success
```

## Quick Start
```bash
    git clone --branch LAB4 https://github.com/pawaris-tangtrakul-23/FRA502-LAB-6674
    cd FRA502-LAB-6674/LAB4_ws/
```
then build (inside LAB4_ws)

```bash
    colcon build && . install/setup.bash
```
### 1. Launch the Complete System
```bash
ros2 launch LAB4 simple_display.launch.py
```

This launches:
- RViz2 with custom configuration
- Robot state publisher with XACRO model
- Control node with all modes enabled
- Random pose service

### 2. Keyboard Teleoperation
```bash
ros2 run LAB4 teleop_jog_keyboard.py
```

**Controls:**
- `w/s`: Move forward/backward (X-axis)
- `a/d`: Move left/right (Y-axis) 
- `r/f`: Move up/down (Z-axis)
- `g`: Toggle control frame (World ↔ End-effector)
- `q/z`: Increase/decrease speed (±10%)
- `Ctrl+C`: Exit

### 3. Service-Based Control

**Inverse Kinematics Mode:**
```bash
ros2 service call /mode robot_interfaces/srv/Mode "{data: 1, position: {x: 0.3, y: 0.1, z: 0.4}}"
```

**Teleoperation Mode:**
```bash
ros2 service call /mode robot_interfaces/srv/Mode "{data: 2}"
```

**Trajectory Mode:**
```bash
ros2 service call /mode robot_interfaces/srv/Mode "{data: 3}"
```

## Topic Interface

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | Current joint positions |
| `/end_effector` | `geometry_msgs/PoseStamped` | End-effector pose (FK) |
| `/target` | `geometry_msgs/PoseStamped` | Target pose visualization |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/teleop_frame` | `std_msgs/String` | Active control frame |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Teleoperation velocity input |
| `/teleop_frame` | `std_msgs/String` | Control frame selection |

## Advanced Features

### Singularity Handling
The system detects kinematic singularities by monitoring the determinant of the position Jacobian:
```python
det_J = np.linalg.det(J_pos)
if abs(det_J) < 0.0001:
    # Stop motion to prevent unstable behavior
```

### Workspace Boundary Enforcement
Real-time workspace validation prevents commands that would move the robot outside safe operating limits:
```python
# Predict next position
next_joint = (self.joint + q_dot * self.dt).tolist()
T_next = self.robot.fkine(next_joint)

# Validate workspace bounds
z_offset = z - 0.2
dist_sq = x**2 + y**2 + z_offset**2
if not (0.03**2 <= dist_sq <= 0.53**2):
    # Reject unsafe motion
```

### Trajectory Generation
Smooth quintic polynomial trajectories ensure continuous motion:
```python
t_array = np.linspace(0, 1.0, 100)
traj = rtb.jtraj(self.joint, q_target, t_array)
```

## Troubleshooting

### Common Issues

**1. "Singularity! Det: X" warnings**
- Normal behavior near kinematic singularities
- Robot will pause motion until moved away from singular configuration

**2. "Workspace limit reached!" warnings**
- Indicates attempted motion outside safe operating envelope
- Use smaller velocity commands or reposition robot

**3. RViz2 not displaying robot**
- Verify XACRO model path in launch file
- Check that `robot_state_publisher` is running
- Ensure `/joint_states` topic is publishing

**4. Keyboard teleoperation not responding**
- Verify terminal has focus
- Check that `teleop_jog_keyboard` node is running
- Confirm `/cmd_vel` topic connection

### Debug Commands

```bash
# Check active nodes
ros2 node list

# Monitor joint states
ros2 topic echo /joint_states

# Verify service availability
ros2 service list

# Check topic connections
ros2 topic info /cmd_vel
```

## Performance Specifications

- **Control Rate**: 100 Hz (10ms update cycle)
- **Trajectory Resolution**: 100 points per 1-second motion
- **IK Convergence**: Levenberg-Marquardt with adaptive damping
- **Workspace Validation**: Real-time boundary checking
- **Safety Response**: <10ms for limit detection and motion stopping

## File Structure

```
├── control.py                 # Main robot controller
├── random_pose.py            # Random pose generation service
├── teleop_jog_keyboard.py    # Keyboard teleoperation interface
├── simple_display.launch.py  # Launch file for complete system
├── jointstate_script.py      # Basic joint state publisher (demo)
└── README.md                 # This documentation
```

## License

This project is licensed under the GNU General Public License v3.0. See the license headers in individual files for details.

---

**Author**: Thanacha Choopojcharoen at CoXsys Robotics (2022)  
**Course**: LAB4 - Robotics Control Systems  
**Framework**: ROS2 Humble, Python 3.10+
