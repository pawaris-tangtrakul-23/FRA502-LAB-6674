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
    # eater_namespace = LaunchConfiguration("eater_namespace")
    # eater_namespace_launch_arg = DeclareLaunchArgument(
    #     "eater_namespace",
    #     default_value="eater2",
    # )
    killer_namespace = "killer1"
    eater_namespace = "eater2"
    # killer_namespace = LaunchConfiguration("killer_namespace")
    # killer_namespace_launch_arg = DeclareLaunchArgument(
    #     "killer_namespace",
    #     default_value="killer1",
    # )
    launch_description.add_action(sampling_frequency_launch_arg)
    # launch_description.add_action(eater_namespace_launch_arg)
    # launch_description.add_action(killer_namespace_launch_arg)

    turtlesim_node = Node(
        package="turtlesim_plus",
        namespace="",
        executable="turtlesim_plus_node.py",
        name="sim1",
    )
    launch_description.add_action(turtlesim_node)

    eater_node = Node(
        package="lab3",
        namespace=eater_namespace,
        executable="eater.py",
        name="eater",
        parameters=[{"sampling_frequency": sampling_frequency}],
    )
    launch_description.add_action(eater_node)

    killer_node = Node(
        package="lab3",
        namespace=killer_namespace,
        executable="killer.py",
        name="killer",
        parameters=[
            {"sampling_frequency": sampling_frequency},
            {"turtle1_name": eater_namespace},
        ],
    )
    launch_description.add_action(killer_node)
    kill_turtle1 = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/remove_turtle ",
            "turtlesim/srv/Kill  ",
            f'"name: turtle1"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(kill_turtle1)

    spawn_killer = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: 1.0, y: 1.0, theta: 0.0,name: {killer_namespace}}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_killer)

    spawn_eater = ExecuteProcess(
        cmd=[
            "ros2 service call ",
            "/spawn_turtle ",
            "turtlesim/srv/Spawn ",
            f'"{{x: 1.0, y: 1.0, theta: 0.0,name: {eater_namespace}}}"',
        ],
        output='screen',
        shell=True,
    )
    launch_description.add_action(spawn_eater)
    return launch_description
