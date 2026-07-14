from launch import LaunchDescription
from launch_ros.actions import Node


thruster_manager_params = {
    "tam.thrusters": [
        "barracuda/thruster_0_joint",
        "barracuda/thruster_1_joint",
        "barracuda/thruster_2_joint",
        "barracuda/thruster_3_joint",
        "barracuda/thruster_4_joint",
        "barracuda/thruster_5_joint",
        "barracuda/thruster_6_joint",
        "barracuda/thruster_7_joint",
    ],
    # 80% of the 30 A board budget per EE (~6 A/thruster); do not raise without EE.
    "tam.min_thrust": -15.0,
    "tam.max_thrust": 20.0,
    # Priority-tiered saturation (heave > roll/pitch > yaw > surge/sway);
    # under saturation the sub gives up surge before depth or attitude:
    #   "priority_allocation": True,
    "control_frame": "barracuda/base_link",
}
# create thruster_manager node
node_thruster_manager = Node(
    package="thruster_manager",
    namespace="barracuda",
    executable="thruster_manager_node",
    output="screen",
    parameters=[thruster_manager_params],
)


def generate_launch_description():
    return LaunchDescription(
        [node_thruster_manager]
    )
