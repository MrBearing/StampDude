"""Launch file for StampDude nodes."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for StampDude nodes."""
    # Declare launch arguments
    enable_twist_arg = DeclareLaunchArgument(
        'enable_twist', default_value='true',
        description='Enable TwistToTwistStamped node')
    enable_point_arg = DeclareLaunchArgument(
        'enable_point', default_value='false',
        description='Enable PointToPointStamped node')
    enable_pose_arg = DeclareLaunchArgument(
        'enable_pose', default_value='false',
        description='Enable PoseToPoseStamped node')
    enable_quaternion_arg = DeclareLaunchArgument(
        'enable_quaternion', default_value='false',
        description='Enable QuaternionToQuaternionStamped node')

    # Create the component container
    container = ComposableNodeContainer(
        name='stamp_dude_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[],
        output='screen'
    )

    # Load composable nodes conditionally
    load_twist_node = LoadComposableNodes(
        target_container='stamp_dude_container',
        composable_node_descriptions=[
            ComposableNode(
                package='stamp_dude',
                plugin='stamp_dude::TwistToTwistStamped',
                name='twist_to_twist_stamped_node'
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_twist'))
    )

    load_point_node = LoadComposableNodes(
        target_container='stamp_dude_container',
        composable_node_descriptions=[
            ComposableNode(
                package='stamp_dude',
                plugin='stamp_dude::PointToPointStamped',
                name='point_to_point_stamped_node'
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_point'))
    )

    load_pose_node = LoadComposableNodes(
        target_container='stamp_dude_container',
        composable_node_descriptions=[
            ComposableNode(
                package='stamp_dude',
                plugin='stamp_dude::PoseToPoseStamped',
                name='pose_to_pose_stamped_node'
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_pose'))
    )

    load_quaternion_node = LoadComposableNodes(
        target_container='stamp_dude_container',
        composable_node_descriptions=[
            ComposableNode(
                package='stamp_dude',
                plugin='stamp_dude::QuaternionToQuaternionStamped',
                name='quaternion_to_quaternion_stamped_node'
            )
        ],
        condition=IfCondition(LaunchConfiguration('enable_quaternion'))
    )

    return LaunchDescription([
        enable_twist_arg,
        enable_point_arg,
        enable_pose_arg,
        enable_quaternion_arg,
        container,
        load_twist_node,
        load_point_node,
        load_pose_node,
        load_quaternion_node
    ])
