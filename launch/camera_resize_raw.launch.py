from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("top_id", default_value="DEV_1AB22C020237"),
        DeclareLaunchArgument("camera_fps", default_value="2.0"),
        DeclareLaunchArgument(
            "top_calib_path",
            default_value="/home/anyone/config/allied_top_camera_calib.yml",
        ),
        DeclareLaunchArgument("save", default_value="True"),
        DeclareLaunchArgument(
            "log_level",
            default_value="INFO",
        ),
    ]

    alliedvision_node = Node(
        namespace="",
        package="alliedvision",
        executable="alliedvision_control",
        name="camera_node",
        parameters=[
            {
                "camera_id_top": LaunchConfiguration("top_id"),
                "camera_fps": LaunchConfiguration("camera_fps"),
                "top_intrinsics": LaunchConfiguration("top_calib_path"),
                "img_save_path": "/home/anyone/Pictures/hires_images",
                "save_images_fps": 0.1,
                "save_top_images": LaunchConfiguration("save"),
            }
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    resize_top_node = ComposableNode(
        name="resize_top_node",
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ResizeNode",
        parameters=[
            {"keep_aspect_ratio": True},
            {"output_height": 200},
            {"output_width": 320},
        ],
        remappings=[
            ("image", "/front/top/image"),
            ("camera_info", "/front/top/camera_info"),
            ("resize/image", "/front/top/image_resize"),
            ("resize/camera_info", "/front/top/camera_resize_info"),
        ],
    )

    resize_bottom_node = ComposableNode(
        name="resize_bottom_node",
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ResizeNode",
        parameters=[
            {"keep_aspect_ratio": True},
            {"output_height": 200},
            {"output_width": 320},
        ],
        remappings=[
            ("image", "/front/bottom/image"),
            ("camera_info", "/front/bottom/camera_info"),
            ("resize/image", "/front/bottom/image_resize"),
            ("resize/camera_info", "/front/bottom/camera_resize_info"),
        ],
    )

    camera_container = ComposableNodeContainer(
        name="camera_container",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[resize_top_node, resize_bottom_node],
        namespace="",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(arguments + [alliedvision_node, camera_container])
