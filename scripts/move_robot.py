#!/usr/bin/env python

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
import time

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ros2_prarob_interfaces.msg import PlannedJointSequence

from kinematics import Kinematics

class prarobClientNode(Node):
    def __init__(self):
        super().__init__('prarob_client_node')

        # Define publisher
        self.robot_goal_publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.joint_sequence_subscriber_ = self.create_subscription(
            PlannedJointSequence,
            '/planned_joint_sequence',
            self.joint_sequence_callback,
            10
        )

        # try:
        #     q1, q2, q3 = Kinematics().inverse_kinematics((0.0, 0.2, 0.02))
        #     self.get_logger().info(f'Inverse kinematics result: q1={q1}, q2={q2}, q3={q3}')
        #     self.move_robot([q1, q2, q3])
        #     time.sleep(1.0)
        #     q1, q2, q3 = Kinematics().inverse_kinematics((0.15, 0.15, 0.0))
        #     self.get_logger().info(f'Inverse kinematics result: q1={q1}, q2={q2}, q3={q3}')
        #     self.move_robot([q1, q2, q3])
        #     time.sleep(1.0)
        #     q1, q2, q3 = Kinematics().inverse_kinematics((-0.15, 0.15, 0.0))
        #     self.get_logger().info(f'Inverse kinematics result: q1={q1}, q2={q2}, q3={q3}')
        #     self.move_robot([q1, q2, q3])
        #     time.sleep(1.0)
        # except Exception as e:
        #     self.get_logger().error(f'Error in inverse kinematics: {e}')

        self.get_logger().info("PraRob Client Node has been started.")



    def move_robot(self, q):
       
        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names.append('joint1')
        goal_trajectory.joint_names.append('joint2')
        goal_trajectory.joint_names.append('joint3')

        goal_point = JointTrajectoryPoint()
        goal_point.positions.append(q[0])
        goal_point.positions.append(q[1])
        goal_point.positions.append(q[2])
        goal_point.time_from_start = Duration(seconds=1.0).to_msg()

        goal_trajectory.points.append(goal_point)

        return self.robot_goal_publisher_.publish(goal_trajectory)
        
    
    def joint_sequence_callback(self, msg):
        self.get_logger().info(f'Received joint sequence: {msg.joints_sequence}')

        goal_trajectory = JointTrajectory()
        goal_trajectory.joint_names.append('joint1')
        goal_trajectory.joint_names.append('joint2')
        goal_trajectory.joint_names.append('joint3')

        time_from_start = msg.duration

        for q in msg.joints_sequence:
            goal_point = JointTrajectoryPoint()
            goal_point.positions.append(q.q1)
            goal_point.positions.append(q.q2)
            goal_point.positions.append(q.q3)
            goal_point.time_from_start = Duration(seconds=time_from_start).to_msg()
            
            goal_trajectory.points.append(goal_point)
            time_from_start += msg.duration

        self.robot_goal_publisher_.publish(goal_trajectory)


def main(args=None):
    rclpy.init(args=args)
    node = prarobClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()