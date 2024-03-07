from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    address_arg = DeclareLaunchArgument(
        "address",
        default_value="192.168.1.1",
        description="IP address of EthernetDAQ box",
    )
    rate_arg = DeclareLaunchArgument(
        "rate",
        default_value="1000",
        description="Publish rate and Ethernet DAQ speed (in hertz)",
    )
    filter_arg = DeclareLaunchArgument(
        "filter", default_value="4", description="Set filtering"
    )

    etherdaq_node = Node(
        package="optoforce_etherdaq_ros2_driver",
        executable="etherdaq_node",
        name="etherdaq_node",
        parameters=[
            {
                "address": LaunchConfiguration("address"),
                "rate": LaunchConfiguration("rate"),
                "filter": LaunchConfiguration("filter"),
            }
        ],
        output="screen",
        # prefix=["xterm -e gdb -ex run --args"],
    )

    etherdaq_subscriber = Node(
        package="optoforce_etherdaq_ros2_driver",
        executable="etherdaq_subscriber",
        name="etherdaq_subscriber",
        output="screen",
    )

    return LaunchDescription(
        [address_arg, rate_arg, filter_arg, etherdaq_node, etherdaq_subscriber]
    )
