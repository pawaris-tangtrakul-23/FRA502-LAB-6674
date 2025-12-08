# ROS2 Turtlesim Pizza Mission Controller

A ROS2 package that implements a multi-phase turtle simulation involving pizza collection, path recording, swarm coordination, and automated cleanup across two Turtlesim environments.

## System Architecture

![alt text](<sim1sim1.png>)


### Nodes

- /sim1/sim1
- /sim2/sim2
- /sim1/teleoper
- /sim2/copy
- /teleop_twist_keyboard

### Topics

- /eraser_spawn_pose
- /sim1/eraser/cmd_vel
- /sim1/eraser/pizza_count
- /sim1/eraser/pose
- /sim1/mouse_position
- /sim1/set_max_pizza
- /sim1/teleoper/cmd_vel
- /sim1/teleoper/pizza_count
- /sim1/teleoper/pose
- /sim1/teleoper/scan
- /sim2/Foxy/cmd_vel
- /sim2/Foxy/pizza_count
- /sim2/Foxy/pose
- /sim2/Foxy/scan
- /sim2/Humble/cmd_vel
- /sim2/Humble/pizza_count
- /sim2/Humble/pose
- /sim2/Humble/scan
- /sim2/Iron/cmd_vel
- /sim2/Iron/pizza_count
- /sim2/Iron/pose
- /sim2/Iron/scan
- /sim2/Noetic/cmd_vel
- /sim2/Noetic/pizza_count
- /sim2/Noetic/pose
- /sim2/Noetic/scan
- /sim2/eraser/cmd_vel
- /sim2/eraser/pizza_count
- /sim2/eraser/pose
- /sim2/mouse_position

### Services

- /sim1/eraser/eat
- /sim1/remove_turtle
- /sim1/sim1/describe_parameters
- /sim1/sim1/get_parameter_types
- /sim1/sim1/get_parameters
- /sim1/sim1/list_parameters
- /sim1/sim1/set_parameters
- /sim1/sim1/set_parameters_atomically
- /sim1/spawn_pizza
- /sim1/spawn_turtle
- /sim1/teleoper/describe_parameters
- /sim1/teleoper/eat
- /sim1/teleoper/get_parameter_types
- /sim1/teleoper/get_parameters
- /sim1/teleoper/list_parameters
- /sim1/teleoper/set_parameters
- /sim1/teleoper/set_parameters_atomically
- /sim1/teleoper/stop
- /sim2/Foxy/eat
- /sim2/Foxy/stop
- /sim2/Humble/eat
- /sim2/Humble/stop
- /sim2/Iron/eat
- /sim2/Iron/stop
- /sim2/Noetic/eat
- /sim2/Noetic/stop
- /sim2/copy/describe_parameters
- /sim2/copy/get_parameter_types
- /sim2/copy/get_parameters
- /sim2/copy/list_parameters
- /sim2/copy/set_parameters
- /sim2/copy/set_parameters_atomically
- /sim2/eraser/eat
- /sim2/order_to_copy
- /sim2/remove_turtle
- /sim2/sim2/describe_parameters
- /sim2/sim2/get_parameter_types
- /sim2/sim2/get_parameters
- /sim2/sim2/list_parameters
- /sim2/sim2/set_parameters
- /sim2/sim2/set_parameters_atomically
- /sim2/spawn_pizza
- /sim2/spawn_turtle
- /teleop_twist_keyboard/describe_parameters
- /teleop_twist_keyboard/get_parameter_types
- /teleop_twist_keyboard/get_parameters
- /teleop_twist_keyboard/list_parameters
- /teleop_twist_keyboard/set_parameters
- /teleop_twist_keyboard/set_parameters_atomically

## 📋 Features

- **Dual Simulation Environment**: Operates across two Turtlesim windows (`sim1` and `sim2`)
- **Interactive Turtle Control**: Keyboard-based teleop control for turtle movement
- **Pizza Management System**: Spawn and collect pizzas with customizable limits
- **Path Recording & Replay**: Record turtle paths and replay them with a swarm of turtles
- **Automated Mission Phases**: Seamless transition between recording, replay, and cleanup phases
- **Dynamic Configuration**: Adjustable controller gains and pizza limits via ROS2 services
- **Automatic Shutdown**: Self-terminating nodes upon mission completion

## 🔧 Requirements

- ROS2 (tested with Humble/Iron)
- Python 3
- Turtlesim package
- Custom interfaces package

## 🛠️ Installation & Build

1. **Clone the package** into your workspace `src` folder:
```bash
cd
git clone --branch exam1 https://github.com/pawaris-tangtrakul-23/FRA502-LAB-6674.git
# Copy or clone the exam1 package here
```

2. **Build the workspace**:
```bash
cd FRA502-LAB-6674/
colcon build 
source install/setup.bash
```

3. **Make script executable** (if necessary):
```bash
chmod +x src/exam1/exam1/teleop_twist_keyboard.py
```

## 🚀 How to Run

### 1. Launch the Simulation
Start both Turtlesim windows, spawn turtles, and initialize all nodes:
```bash
ros2 launch exam1 launch.py
```

### 2. Run the Controller
Open a new terminal and run the keyboard controller with proper topic remapping:
```bash
# Source the workspace first
source ~/FRA502-LAB-6674/install/setup.bash

# Run the teleop controller with CLI arguments
ros2 run exam1 teleop_twist_keyboard.py --ros-args \
  -p cmd_vel_topic:=/sim1/teleoper/cmd_vel \
  -p service_name:=/order
```

## 🎮 Controls & Gameplay

### Keyboard Mappings
| Key | Action |
|-----|--------|
| `W` | Move forward |
| `A` | Turn left |
| `S` | Move backward |
| `D` | Turn right |
| `P` | Spawn pizza at current location |
| `R` | Save current pizza path |
| `E` | Eat pizza (manual control if needed) |

### Mission Workflow

#### Phase 1: Recording (Sim 1)
1. Use `W/A/S/D` to move the Teleoper turtle
2. Press `P` to place pizzas at desired locations
3. Press `R` to save the current path
4. **Repeat this process 4 times** to record 4 different paths

#### Phase 2: Replay (Sim 2)
1. After saving the 4th path, the "Copy Swarm" automatically activates
2. Four turtles (Foxy, Noetic, Humble, Iron) spawn in Sim 2
3. Each turtle follows one of your recorded paths, spawning pizzas along the way

#### Phase 3: Cleanup
1. **Click anywhere** in the Sim 2 window after the swarm finishes
2. The Eraser Turtle spawns and destroys the swarm
3. Eraser travels through a "portal" (your click location)
4. Eraser jumps to Sim 1 and destroys the Teleoper turtle

#### Phase 4: Shutdown
- All nodes automatically terminate upon mission completion

## ⚙️ Configuration

### Change Maximum Pizza Limit
Adjust the number of pizzas you can spawn before needing to save:
```bash
ros2 service call /sim1/teleoper/set_max_pizza \
  custom_interfaces/srv/SetMaxPizza "{max_pizza: {data: 20}}"
```

### Adjust Controller Gains
Modify the Linear and Angular proportional gains for the controller:
```bash
ros2 service call /sim1/teleoper/set_eater_kp \
  custom_interfaces/srv/SetParam "{kp_linear: {data: 5.0}, kp_angular: {data: 10.0}}"
```


## 🐛 Troubleshooting

- **Permission denied**: Ensure the Python script is executable with `chmod +x`
- **Topic not found**: Verify the topic remapping arguments are correct
- **Service unavailable**: Make sure the launch file has fully started before running the controller
- **Turtlesim not responding**: Check that both simulation windows are properly focused

## 📝 Notes

- The controller uses CLI arguments for topic remapping to avoid hardcoding
- The system automatically handles turtle spawning and cleanup
- Mission progress is tracked automatically across all phases
- Click precisely in Sim 2 during the cleanup phase for optimal eraser positioning
