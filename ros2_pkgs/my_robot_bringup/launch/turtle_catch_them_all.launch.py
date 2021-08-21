from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim"
    )

    turtle_spawner_node = Node(
        package="turtle_catch_them_all",
        executable="turtle_spawner",
        name="turtle_spawner",
        parameters=[
            {"spawn_frequency": 1.5},
            {"turtle_name_prefix": "my_turtle_"}
        ]
    )

    turtle_controller_node = Node(
        package="turtle_catch_them_all",
        executable="turtle_controller",
        name="turtle_controller",
        parameters=[
            {"catch_closest_turtle_fist": True}
        ]
    )

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)
    return ld
