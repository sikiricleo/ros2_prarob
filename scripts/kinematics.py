#!/usr/bin/env python3

import math
import numpy
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

#modules and XL-430 dimensions
link_RR = 0.20
link_90 = 0.016
izvrsni_clan = 0.023
postolje = 0.06
dynamixel_height = 0.03525
dynamixel_width = 0.036
dynamixel_height_from_bolt = 0.032
dynamixel_offset = 0.012
tool_length = 0


#link legths of
link1 = postolje + dynamixel_width + link_90 + dynamixel_height_from_bolt
link2 = link_RR
link3 = dynamixel_height + izvrsni_clan + tool_length


class Kinematics(Node):

    def __init__(self):
        super().__init__('prarob_kinematics_node')
        

        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        self.joint_state = None

        self.br = TransformBroadcaster(self)

        self.get_logger().info("Kinematics Node has been started.")


    def direct_kinematics(self, theta1, theta2, theta3):
        link1_matrix = numpy.array([[0,          numpy.cos(theta1),  -numpy.sin(theta1),        0            ], 
                                    [0,          numpy.sin(theta1),   numpy.cos(theta1),  dynamixel_offset   ], 
                                    [1,                     0,               0,               link1          ], 
                                    [0,                     0,               0,                 1            ]
                                    ])
        link2_matrix = numpy.array([[numpy.cos(theta2), -numpy.sin(theta2),  0,     link2 * numpy.cos(theta2)], 
                                    [numpy.sin(theta2),  numpy.cos(theta2),  0,     link2 * numpy.sin(theta2)], 
                                    [0,                     0,               1,                 0            ], 
                                    [0,                     0,               0,                 1            ]
                                    ])
        
        link3_matrix = numpy.array([[numpy.cos(theta3), -numpy.sin(theta3),  0,     link3 * numpy.cos(theta3)], 
                                    [numpy.sin(theta3),  numpy.cos(theta3),  0,     link3 * numpy.sin(theta3)], 
                                    [0,                     0,               1,                 0            ], 
                                    [0,                     0,               0,                 1            ]
                                    ])
        
        T_base_tool = numpy.dot(numpy.dot(link1_matrix, link2_matrix), link3_matrix)
        return T_base_tool

    @staticmethod
    def inverse_kinematics(self, position):
        x = position[0] 
        y = position[1]
        z = position[2]

        theta1 = math.atan2(y, x)

        r = math.sqrt(x**2 + y**2)
        z_offset = z - link1

        D = (r**2 + z_offset**2 - link2**2 - link3**2) / (2 * link2 * link3)
        if abs(D) > 1:
            raise ValueError("Kuda ćeš sinko?")

        theta3 = math.atan2(-math.sqrt(1 - D**2), D)
        theta2 = math.atan2(z_offset, r) - math.atan2(link3 * math.sin(theta3), link2 + link3 * math.cos(theta3))

        return theta1, theta2, theta3
    
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