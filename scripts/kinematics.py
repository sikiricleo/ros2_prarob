#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from sympy import symbols, sin, cos, nsolve

# Dimensions (meters)
link_RR = 0.20
link_90 = 0.016
izvrsni_clan = 0.023
postolje = 0.006
dynamixel_height = 0.03525
dynamixel_width = 0.036
dynamixel_height_from_bolt = 0.032
dynamixel_offset = 0.012
tool_length = 0.107


p0x = 0.012
p0z = postolje + 0.033

p1x = 0.019
p1z = 0.02 + 0.028

p2x = link_RR

p3xDK = + 0.04 + 0.02 + tool_length
p3xIK = 0.04 + 0.02 + tool_length
p3z = 0.02


joint_state = None

class Kinematics(Node):

    def __init__(self):
        super().__init__('prarob_kinematics_node')
        

        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        self.br = TransformBroadcaster(self)        

        self.get_logger().info("Kinematics Node has been started.")
    
    def wrap_angle(theta):
        return (theta + math.pi) % (2 * math.pi) - math.pi
    
    @staticmethod
    def direct_kinematics(theta1, theta2, theta3):
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
            [ 0, 0, 1, p3xDK],
            [ 0, 1, 0, 0],
            [-1, 0, 0, p3z],
            [ 0, 0, 0, 1]
        ])

        Transform_matrix = A0_1 @ A1_2 @ A2_3 @ A3_4
        return Transform_matrix

    # @staticmethod
    # def inverse_kinematics(w):
    #     w1, w2, w3, w4, w5, w6 = w

    #     theta1 = math.atan2(w4, -w5)

    #     theta2_3 = math.atan2(w4, math.sin(theta1) * w6)

    #     theta2 = -math.acos((w3 - p0z - p1z - p3x * math.cos(theta2_3))/p2x)

    #     # if(abs(theta2 - joint_state.position[1]) > abs(-theta2 - joint_state.position[1])):
    #     #     theta2 = -theta2

    #     theta3 = - theta2_3 + theta2

    #     return theta1, theta2, theta3

    @staticmethod
    def inverse_kinematics(w):
        x_target, y_target, z_target = w


        theta1, theta2, theta3 = symbols('theta1 theta2 theta3')

        wx = p0x + p1x*cos(theta1) - p3z*cos(theta1) + p2x*sin(theta1)*sin(theta2) - p3xIK*cos(theta2)*sin(theta1)*sin(theta3) + p3xIK*cos(theta3)*sin(theta1)*sin(theta2)
        wy = p1x*sin(theta1) - p3z*sin(theta1) - p2x*cos(theta1)*sin(theta2) - p3xIK*sin(theta2-theta3)*cos(theta1)
        wz = p0z + p1z + p2x*cos(theta2) + p3xIK*cos(theta2-theta3)
        wrzt = cos(theta2 - theta3)

        eq1 = wx - x_target
        eq2 = wy - y_target
        eq3 = wz - z_target

        initial_guess = [0, -math.pi/4, math.pi/2]
        try: 
            solutions = nsolve((eq1, eq2, eq3), (theta1, theta2, theta3), initial_guess)
            theta1_num, theta2_num, theta3_num = [Kinematics.wrap_angle(float(s)) for s in solutions]
            T_base_tool = Kinematics.direct_kinematics(theta1_num, theta2_num, theta3_num)
            if not (np.isclose(T_base_tool[0, 3], x_target, atol=5e-3) and
                    np.isclose(T_base_tool[1, 3], y_target, atol=5e-3) and
                    np.isclose(T_base_tool[2, 3], z_target, atol=5e-3)):
                raise ValueError("Inverse kinematics solution does not match target position.")

        except Exception as e:
            raise ValueError("Could not find a valid solution for the inverse kinematics.")

        return theta1_num, theta2_num, theta3_num
    
    def joint_state_callback(self, msg):
        joint_state = msg
       
        theta1 = joint_state.position[0]
        theta2 = joint_state.position[1]
        theta3 = joint_state.position[2]

        T_base_tool = Kinematics.direct_kinematics(theta1, theta2, theta3)

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