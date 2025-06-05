#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Dimensions (meters)
link_RR = 0.20
link_90 = 0.016
izvrsni_clan = 0.023
postolje = 0.006
dynamixel_height = 0.03525
dynamixel_width = 0.036
dynamixel_height_from_bolt = 0.032
dynamixel_offset = 0.012
tool_length = 0 #0.107


p0x = 0.012
p0z = postolje + 0.033

p1x = 0.019
p1z = 0.02 + 0.028

p2x = link_RR

p3x = + 0.04 + 0.02 + tool_length
p3z = 0.02




class Kinematics(Node):

    def __init__(self):
        super().__init__('prarob_kinematics_node')
        

        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.joint_state = None

        self.br = TransformBroadcaster(self)        

        self.get_logger().info("Kinematics Node has been started.")


    def direct_kinematics(self, theta1, theta2, theta3):
        A0_1 = np.array([
            [np.cos(theta1), -np.sin(theta1), 0, p0x],
            [np.sin(theta1),  np.cos(theta1), 0, 0],
            [0,               0,              1, p0z],
            [0,               0,              0, 1]
        ])

        A1_2 = np.array([
            [0,               0,                1, p1x],
            [-np.sin(theta2), -np.cos(theta2),  0, 0],
            [np.cos(theta2),  -np.sin(theta2),  0, p1z],
            [0,               0,                0, 1]
        ])

        A2_3 = np.array([
            [np.cos(theta3),  -np.sin(theta3),  0, p2x],
            [-np.sin(theta3), -np.cos(theta3),  0, 0],
            [0,               0,              -1,  0],
            [0,               0,               0,  1]
        ])

        A3_4 = np.array([
            [ 0, 0, 1, p3x],
            [ 0, 1, 0, 0],
            [-1, 0, 0, p3z],
            [ 0, 0, 0, 1]
        ])

        Transform_matrix = A0_1 @ A1_2 @ A2_3 @ A3_4
        return Transform_matrix

    # @staticmethod
    # def inverse_kinematics(position):
    #     Px = position[0] 
    #     Py = position[1]
    #     Pz = position[2]

    #     theta1 = np.arctan2(Py, Px)

    #     r = np.sqrt(Px**2 + Py**2)
    #     z = Pz - link1 - tool_link

    #     L2 = link2
    #     L3 = link3

    #     D = (r**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3)
    #     if abs(D) > 1:
    #         #self.get_logger().warn("Kuda ćeš sinko?")
    #         return None

    #     theta3 = np.arctan2(np.sqrt(1 - D**2), D)

    #     phi = np.arctan2(z, r)
    #     beta = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    #     theta2 = phi - beta

    #     return theta1, theta2, theta3
    
    def joint_state_callback(self, msg):
        self.joint_state = msg
       
        theta1 = self.joint_state.position[0]
        theta2 = self.joint_state.position[1]
        theta3 = self.joint_state.position[2]

        T_base_tool = self.direct_kinematics(theta1, theta2, theta3)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'link1'
        t.child_frame_id = 'direct_kinematics'
        t.transform.translation.x = T_base_tool[0, 3]
        t.transform.translation.y = T_base_tool[1, 3]
        t.transform.translation.z = T_base_tool[2, 3]
        
        rotation = Rotation.from_matrix(T_base_tool[:3, :3])
        q = rotation.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]       
        t.transform.rotation.w = q[3]

        self.br.sendTransform(t)

            


def main(args=None):
    rclpy.init(args=args)
    node = Kinematics()
    rclpy.spin(node)
    rclpy.shutdown()

    
if __name__=='__main__':
    main()