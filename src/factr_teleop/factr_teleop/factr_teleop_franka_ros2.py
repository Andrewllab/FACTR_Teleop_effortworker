
import numpy as np

import rclpy
from sensor_msgs.msg import JointState

from factr_teleop.factr_teleop import FACTRTeleop



class FACTRTeleopFrankaROS2(FACTRTeleop):

    def __init__(self):
        super().__init__()
        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]

        self.follower_external_torque = np.zeros(7)
        self.follower_velocity = np.zeros(7)
        self.follower_gripper_torque = 0.0

        self.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

    def set_up_communication(self):
        
        # create joint and gripper command publishers to follower arm
        self.joint_pos_cmd_pub = self.create_publisher(JointState, '/factr_teleop/joint_cmd', 10)
        self.gripper_pos_cmd_pub = self.create_publisher(JointState, '/factr_teleop/gripper_cmd', 10)

        # create follower joint pos, vel and external torque subscribers
        self.follower_joint_state_sub = self.create_subscription(
            JointState,
            '/franka_robot_state_broadcaster/measured_joint_states',
            self._follower_state_callback,
            1
        )

        if self.enable_torque_feedback:
            # create external torque subscriber from follower arm
            self.follower_torque_sub = self.create_subscription(
                JointState,
                '/franka_robot_state_broadcaster/external_joint_torques',
                self._follower_torque_callback,
                1
            )

        if self.enable_gripper_feedback:
            # create gripper torque subsriber from follower arm
            self.gripper_pub = self.create_subscription(
                JointState, 
                '/franka_state_controller/gripper_torque', 
                self._follower_gripper_torque_callback, 
                10
        )

    def _follower_state_callback(self,msg):  # ?
        self.follower_position = np.array(msg.position)

    def _follower_torque_callback(self, msg):
        self.follower_external_torque = np.array(msg.effort)

    def _gripper_torque_callback(self, msg):
        gripper_torque = np.array(msg.effort)
        self.follower_gripper_torque = gripper_torque[0]
    

    def get_leader_gripper_feedback(self):
        return self.follower_gripper_torque
    
    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0*gripper_feedback / self.gripper_feedback_gain
        return torque_gripper

    def get_leader_arm_external_joint_torque(self):
        return self.follower_external_torque

    def update_communication(self, leader_arm_pos, leader_gripper_pos,): # leader_arm_vel, leader_arm_torque):

        # publish joint position
        leader_joint_states = JointState()
        leader_joint_states.header.stamp = self.get_clock().now().to_msg()
        leader_joint_states.name = self.joint_names
        leader_joint_states.header.frame_id = "fr3_base_link"
        leader_joint_states.position = list(map(float, leader_arm_pos))
        self.joint_pos_cmd_pub.publish(leader_joint_states)

        # publish gripper state
        leader_gripper_states = JointState()
        leader_gripper_states.header.stamp = self.get_clock().now().to_msg()
        leader_gripper_states.name = ["gripper"]
        leader_gripper_states.position = list(map(float, [leader_gripper_pos]))
        self.gripper_pos_cmd_pub.publish(leader_gripper_states)


def main(args=None):
    rclpy.init(args=args)

    factr_node = FACTRTeleopFrankaROS2()

    try:
        while rclpy.ok():
            rclpy.spin(factr_node)
    except KeyboardInterrupt:
        factr_node.get_logger().info("Keyboard interrupt received. Shutting down...")
        factr_node.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()