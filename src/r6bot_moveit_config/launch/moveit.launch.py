import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 绝对路径定义 (请确保路径与你的虚拟机一致)
    ws_path = "/home/elk/ROS2-VPG_ws"
    urdf_path = os.path.join(ws_path, "src/ros2_control_demos/example_7/description/urdf/r6bot.urdf.xacro")
    srdf_path = os.path.join(ws_path, "src/r6bot_moveit_config/config/r6bot.srdf")
    kinematics_path = os.path.join(ws_path, "src/r6bot_moveit_config/config/kinematics.yaml")
    limits_path = os.path.join(ws_path, "src/r6bot_moveit_config/config/joint_limits.yaml")

    # 2. 处理 URDF (Xacro)
    robot_description_config = xacro.process_file(urdf_path)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # 3. 处理 SRDF
    with open(srdf_path, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": semantic_content}

    # 4. 处理 Kinematics YAML
    with open(kinematics_path, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # 5. 处理 Joint Limits YAML
    with open(limits_path, 'r') as f:
        joint_limits_config = yaml.safe_load(f)

    # 合并参数
    moveit_controllers = {
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller"],
            "arm_controller": {
                "type": "FollowJointTrajectory",
                "action_ns": "follow_joint_trajectory",
                "default": True,
                "joints": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            },
        }
    }

    # 汇总所有参数给 MoveGroup 和 RViz
    all_params = [
        robot_description,
        robot_description_semantic,
        kinematics_config,
        joint_limits_config,
        {"publish_robot_description": True,
         "publish_robot_description_semantic": True,
         "publish_planning_scene": True}
    ]

    return LaunchDescription([
        # MoveGroup 核心节点
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=all_params,
        ),
        # 机器人状态发布
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        # 关节状态发布 (滑块)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen",
            env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
        # 静态 TF (world -> base_link)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        ),
        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=all_params,
            env={'LIBGL_ALWAYS_SOFTWARE': '1'}
        ),
    ])