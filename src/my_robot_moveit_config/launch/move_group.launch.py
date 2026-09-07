from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "humanoid_wheeled_robot",
        package_name="my_robot_moveit_config"
    ).to_moveit_configs()

    moveit_config.trajectory_execution[
        "moveit_controller_manager"
    ] = "moveit_simple_controller_manager/MoveItSimpleControllerManager"

    moveit_config.trajectory_execution[
        "moveit_simple_controller_manager"
    ] = {
        "controller_names": ["arm_controller"],
        "arm_controller": {
            "type": "FollowJointTrajectory",
            "action_ns": "follow_joint_trajectory",
            "default": True,
            "joints": [
                "shoulder_pitch_joint",
                "shoulder_roll_joint",
                "shoulder_yaw_joint",
                "elbow_pitch_joint",
                "wrist_pitch_joint",
                "wrist_roll_joint",
            ],
        },
    }

    return generate_move_group_launch(moveit_config)
