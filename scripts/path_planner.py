#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
import numpy as np
import time

from ros2_prarob_interfaces.msg import NavTask, JointState, PlannedJointSequence 
from yolo_msgs.msg import DetectionArray
from prarob_calib.camera_to_world import image2world
from kinematics import Kinematics

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.heuristic import euclidean

import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.task_subscriber = self.create_subscription(
            NavTask,
            '/task',
            self.task_callback,
            10
        )

        self.yolo_subscriber = self.create_subscription(
            DetectionArray,
            '/yolo/detections',
            self.detections_callback,
            10
        )

        self.joint_sequence_publisher = self.create_publisher(
            PlannedJointSequence,
            '/planned_joint_sequence',
            10
        )

        self.camera_intrinsics = np.array(
                                 [[1310.9971846209182, 0, 333.73199923763389], 
                                  [0, 1317.9156230092792, 392.66682249055191],
                                  [0, 0, 1]
                                 ])
        self.T_camera_robot = np.array(
                              [[0.98741, 0.12037, 0.10261, 0.01909],
                               [0.10945, -0.98831, 0.10617, 0.06678],
                               [0.11419, -0.0936, -0.98904, 0.92001],
                               [0, 0, 0, 1]
                              ])

        self.yolo_detections = None

        self.inference_id = 0

        self.get_logger().info("Path Planner Node has been started.")
    
    def plot_grid(self, grid, path=None, start=None, goal=None):
            grid_array = np.array(grid)
            fig, ax = plt.subplots(figsize=(6, 6))

            # Show grid (1 = free, 0 = obstacle)
            ax.imshow(grid_array, cmap='gray_r', origin='upper')

            # Plot path if available
            if path:
                path_x = [p[0] for p in path]
                path_y = [p[1] for p in path]
                ax.plot(path_x, path_y, color='blue', linewidth=2, label='Path')

            # Plot start and goal
            if start:
                ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
            if goal:
                ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')

            ax.set_title('Path Planning Grid')
            ax.set_xlabel('Column Index')
            ax.set_ylabel('Row Index')
            ax.legend()
            plt.grid(True)
            
            package_path = get_package_share_directory('ros2_prarob')
            temp_folder = os.path.join(package_path, 'temp')
            os.makedirs(temp_folder, exist_ok=True)

            # Save the image
            image_path = os.path.join(temp_folder, f'path_plot_{self.inference_id}.png')
            plt.savefig(image_path, bbox_inches='tight')
            plt.show()
            self.inference_id += 1


    def detections_callback(self, msg):
        self.yolo_detections = msg.detections

    def yolo_detections_to_world(self, msg):
        start_position = None
        goal_position = None
        obstacle_boundaries = []
        for detection in self.yolo_detections:
            if detection.class_name == msg.start_class_name:
                start_position = detection.bbox.center
            elif detection.class_name == msg.goal_class_name:
                goal_position = detection.bbox.center
            elif detection.class_name in msg.obstacle_class_names:
                obstacle_boundaries.append(
                    (detection.bbox.center.position.x - detection.bbox.size.x / 2,
                    detection.bbox.center.position.y - detection.bbox.size.y / 2,
                    detection.bbox.center.position.x + detection.bbox.size.x / 2,
                    detection.bbox.center.position.y + detection.bbox.size.y / 2)
                )

        if start_position is None or goal_position is None:
            self.get_logger().warn("Start or goal position not found in YOLO detections.")
            return None, None, None, None

        start_position_world = image2world(
            (start_position.position.x, start_position.position.y),
            self.camera_intrinsics,
            self.T_camera_robot
        )
        goal_position_world = image2world(
            (goal_position.position.x, goal_position.position.y),
            self.camera_intrinsics,
            self.T_camera_robot
        )

        if not obstacle_boundaries:
            self.get_logger().warn("No obstacles found in YOLO detections.")
            obstacle_positions_bottom_left = []
            obstacle_positions_top_right = []

        else:
            obstacle_positions_bottom_left = [
                image2world(
                    (obstacle[0], obstacle[1]),
                    self.camera_intrinsics,
                    self.T_camera_robot
                ) for obstacle in obstacle_boundaries
            ]
            obstacle_positions_top_right = [
                image2world(
                    (obstacle[2], obstacle[3]),
                    self.camera_intrinsics,
                    self.T_camera_robot
                ) for obstacle in obstacle_boundaries
            ]
            self.get_logger().info(f"Obstacle positions: {obstacle_positions_bottom_left} to {obstacle_positions_top_right}")

        return start_position_world, goal_position_world, obstacle_positions_bottom_left, obstacle_positions_top_right
    
    def clamp_to_grid(self, num, limit):
        return max(0, min(num, limit))
    
    def plan_path(self, start_position_world, goal_position_world, obstacle_positions_bottom_left, obstacle_positions_top_right):
        grid_resolution = 0.005      # 5 mm resolution
        grid_origin_x = -0.25
        grid_origin_y = 0.35
        num_rows = int(0.4 / grid_resolution)  # 40 cm height
        num_cols = int(0.4 / grid_resolution)  # 40 cm width
        grid = np.ones((num_rows, num_cols)) 

        # Mark obstacles in the grid
        if obstacle_positions_bottom_left:
            inflate = int(0.01 / grid_resolution)  # 2 cm inflation
            for i in range(len(obstacle_positions_bottom_left)):
                x_min = obstacle_positions_bottom_left[i][0]
                y_min = obstacle_positions_bottom_left[i][1]
                x_max = obstacle_positions_top_right[i][0]
                y_max = obstacle_positions_top_right[i][1]
                col_min = self.clamp_to_grid(int((x_min - grid_origin_x) / grid_resolution) - inflate, num_cols - 1)
                col_max = self.clamp_to_grid(int((x_max - grid_origin_x) / grid_resolution) + inflate, num_cols - 1)
                row_min = self.clamp_to_grid(int((grid_origin_y - y_min) / grid_resolution) - inflate, num_rows - 1)
                row_max = self.clamp_to_grid(int((grid_origin_y - y_max) / grid_resolution) + inflate, num_rows - 1)
                
                if row_min > row_max:
                    t = row_min
                    row_min = row_max
                    row_max = t
                if col_min > col_max:
                    t = col_min
                    col_min = col_max
                    col_max = t

                self.get_logger().info(f"Obstacle at grid: from row ({row_min} to {row_max}), from col ({col_min} to {col_max})")

                for r in range(row_min, row_max + 1):
                    for c in range(col_min, col_max + 1):
                        grid[r][c] = 0

        grid_object = Grid(matrix = grid.tolist())

        start_node = grid_object.node(
            self.clamp_to_grid(int((start_position_world[0] - grid_origin_x) / grid_resolution), num_cols - 1),
            self.clamp_to_grid(int((grid_origin_y - start_position_world[1]) / grid_resolution), num_rows - 1)
        )
        self.get_logger().info(f"Start node: {start_node}")

        goal_node = grid_object.node(
            self.clamp_to_grid(int((goal_position_world[0] - grid_origin_x) / grid_resolution), num_cols - 1),
            self.clamp_to_grid(int((grid_origin_y - goal_position_world[1]) / grid_resolution), num_rows - 1)
        )
        self.get_logger().info(f"Goal node: {goal_node}")

        finder = AStarFinder(
            diagonal_movement=DiagonalMovement.always,
            heuristic=euclidean
        )

        path, runs = finder.find_path(
            start_node,
            goal_node,
            grid_object
        )
        
        if not path:
            self.get_logger().warn("No path found.")
            return None
        else:
            self.get_logger().info(f"Path found with {len(path)} nodes and {runs} runs.")

        # Visualize the grid and the path
        grid_copy = [[1 - int(cell) for cell in row] for row in grid]  # Invert: 1 = obstacle for imshow
        path_for_plot = [(node.x, node.y) for node in path]
        self.plot_grid(grid_copy, path=path_for_plot, start=(start_node.x, start_node.y), goal=(goal_node.x, goal_node.y))

        robot_path_coordinates = []
        for node in path:
            robot_x = grid_origin_x + node.x * grid_resolution + grid_resolution / 2 
            robot_y = grid_origin_y - node.y * grid_resolution - grid_resolution / 2
            robot_path_coordinates.append((robot_x, robot_y))

        return robot_path_coordinates

    def task_callback(self, msg):
        self.get_logger().info(f"Received task: Navigate from {msg.start_class_name} to {msg.goal_class_name} while avoiding {msg.obstacle_class_names}.")
        
        if self.yolo_detections is None:
            self.get_logger().warn("No YOLO detections received yet.")
            return
        
        start_position_world, goal_position_world, obstacle_positions_bottom_left, obstacle_positions_top_right = self.yolo_detections_to_world(msg)
        if start_position_world is None or goal_position_world is None:
            return
        
        robot_path_coordinates = self.plan_path(
            start_position_world,
            goal_position_world,
            obstacle_positions_bottom_left,
            obstacle_positions_top_right
        )

        if robot_path_coordinates is None:
            return
        
        try:
            # pen up
            planned_joint_sequence_pu = PlannedJointSequence()
            joint_state_pu = JointState()
            joint_state_pu.q1, joint_state_pu.q2, joint_state_pu.q3 = Kinematics.inverse_kinematics((robot_path_coordinates[0][0], robot_path_coordinates[0][1], 0.02))
            planned_joint_sequence_pu.joints_sequence.append(joint_state_pu)
            planned_joint_sequence_pu.duration = 1.0
            self.joint_sequence_publisher.publish(planned_joint_sequence_pu)

            time.sleep(1.0)

            # pen down
            planned_joint_sequence = PlannedJointSequence()
            for coord in robot_path_coordinates:
                joint_state = JointState()
                joint_state.q1, joint_state.q2, joint_state.q3 = Kinematics.inverse_kinematics((coord[0], coord[1], 0,)) 
                planned_joint_sequence.joints_sequence.append(joint_state)
            planned_joint_sequence.duration = 4.0 / len(robot_path_coordinates)
            self.joint_sequence_publisher.publish(planned_joint_sequence)

            time.sleep(4.0)
        
            # pen up
            planned_joint_sequence_pu = PlannedJointSequence()
            joint_state_pu = JointState()
            joint_state_pu.q1, joint_state_pu.q2, joint_state_pu.q3 = Kinematics.inverse_kinematics((robot_path_coordinates[-1][0], robot_path_coordinates[-1][1], 0.05)) 
            planned_joint_sequence_pu.joints_sequence.append(joint_state_pu)
            joint_state_pu.q1, joint_state_pu.q2, joint_state_pu.q3 = (0.0, 0.0, 0.0)
            planned_joint_sequence_pu.joints_sequence.append(joint_state_pu)
            planned_joint_sequence_pu.duration = 1.0
            self.joint_sequence_publisher.publish(planned_joint_sequence_pu)
        
        except Exception as e:
            self.get_logger().error(f"Error in planning path: {e}")
            return

    

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()