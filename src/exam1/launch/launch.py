from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    launch_description = LaunchDescription()
    
    sampling_frequency = LaunchConfiguration("sampling_frequency")
    sampling_frequency_launch_arg = DeclareLaunchArgument(
        "sampling_frequency",
        default_value="100.0",
    )
    launch_description.add_action(sampling_frequency_launch_arg)

    # 1. Start Turtlesim in Namespace sim1
    turtlesim_node1 = Node(
        package="turtlesim_plus",
        namespace="sim1",
        executable="turtlesim_plus_node.py",
        name="sim1",
    )
    launch_description.add_action(turtlesim_node1)

    # 2. Start Turtlesim in Namespace sim2
    turtlesim_node2 = Node(
        package="turtlesim_plus",
        namespace="sim2",
        executable="turtlesim_plus_node.py",
        name="sim2",
    )
    launch_description.add_action(turtlesim_node2)

    # 3. Teleoper Node in sim1
    # CRITICAL FIX: Remap 'order' to Global '/order' so Keyboard can find it
    teleoper_node = Node(
        package="exam1",
        namespace="sim1",
        executable="teleoper_node.py",
        name="teleoper",
        parameters=[{"sampling_frequency": sampling_frequency}],
        remappings=[
            ('order_to_copy', '/sim2/order_to_copy'), # Call Sim2 service
            ('order', '/order') # Expose service to GLOBAL namespace
        ]
    )
    launch_description.add_action(teleoper_node)

    # 4. Copy Node in sim2
    # CRITICAL FIX: Remap 'order' to Global '/order' to talk to Teleoper
    copy_node = Node(
        package="exam1",
        namespace="sim2",
        executable="copy_node.py",
        name="copy",
        parameters=[{"sampling_frequency": sampling_frequency}],
        remappings=[
            ('order', '/order'), # Call GLOBAL service (Teleoper)
            ('order_to_copy', 'order_to_copy') # Local service
        ]
    )
    launch_description.add_action(copy_node)

    # 5. Service Calls for Spawning Turtles
    
    kill_sim1_turtle1 = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim1/remove_turtle",
            "turtlesim/srv/Kill  ",
            f'"name: turtle1"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(kill_sim1_turtle1)

    kill_sim2_turtle1 = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim2/remove_turtle",
            "turtlesim/srv/Kill  ",
            f'"name: turtle1"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(kill_sim2_turtle1)

    spawn_teleoper = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim1/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: -5.5, y: -5.5, theta: 0.0,name: teleoper}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_teleoper)

    spawn_Foxy = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim2/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: -5.5, y: -5.5, theta: 0.0,name: Foxy}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_Foxy)

    spawn_Noetic = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim2/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: -5.5, y: -5.5, theta: 0.0,name: Noetic}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_Noetic)

    spawn_Humble = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim2/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: -5.5, y: -5.5, theta: 0.0,name: Humble}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_Humble)

    spawn_Iron = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/sim2/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: -5.5, y: -5.5, theta: 0.0,name: Iron}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_Iron)

    return launch_description