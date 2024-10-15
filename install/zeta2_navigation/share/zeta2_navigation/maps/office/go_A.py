#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from PIL import Image
import yaml
from tf_transformations import quaternion_from_euler  # 오일러 각도 변환을 위한 임포트

# 픽셀 좌표를 월드 좌표로 변환하는 함수
def pixel_to_world(u, v, width, height, resolution, origin):
    x = u * resolution + origin[0]
    y = (height - v) * resolution + origin[1]
    return x, y

# 맵 데이터를 로드하는 함수
def load_map_data(pgm_file, yaml_file):
    image = Image.open(pgm_file)
    width, height = image.size

    with open(yaml_file, 'r') as file:
        map_metadata = yaml.safe_load(file)

    resolution = map_metadata['resolution']
    origin = map_metadata['origin']

    return width, height, resolution, origin

class NavigationWithObstacle(Node):
    def __init__(self):
        super().__init__('navigation_with_obstacle')
        self.navigator = BasicNavigator()
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.obstacle_detected = False

        # Load map data
        pgm_file = 'map1.pgm'
        yaml_file = 'mapping.yaml'
        self.width, self.height, self.resolution, self.origin = load_map_data(pgm_file, yaml_file)

        # Define the full fixed route with all waypoints
        self.fixed_route = [
            # (135, 79),  # Initial Point
            # (147, 79),  # Intermediate Point
            (146, 65)  # 식당
            (166, 65),  # Intermediate Point
            (160, 85)  # Hyeondong Hall
            # (173, 84),  # Nehemiah Hall
            # (171, 54),  # Intermediate Point
            # (186, 54),   # Oseok Hall
        ]

        # Convert fixed route pixel coordinates to goal poses
        self.goal_poses = self.get_goal_poses()

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        
        for wp in self.goal_poses:  # 모든 경로 포인트를 누적해서 발행
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
            pose.pose.position.x = wp.pose.position.x
            pose.pose.position.y = wp.pose.position.y
            pose.pose.orientation = wp.pose.orientation
            path.poses.append(pose)
        
        self.path_pub.publish(path)  # 전체 경로를 한 번에 발행
        self.get_logger().info('전체 경로 발행됨.')  # 경로 발행 로그 추가


    def lidar_callback(self, scan_data):
        min_distance = min(scan_data.ranges)
        safe_distance = 0.5  # Set safe distance
        if min_distance < safe_distance:
            self.obstacle_detected = True
            self.navigator.cancelTask()  # Cancel navigation task
            self.get_logger().info('장애물 감지! 로봇 멈춤.')
        else:
            self.obstacle_detected = False

    def get_goal_poses(self):
        goal_poses = []
        for u, v in self.fixed_route:
            x, y = pixel_to_world(u, v, self.width, self.height, self.resolution, self.origin)
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y

            # 쿼터니언 값으로 각도를 변환하여 설정
            q = quaternion_from_euler(0, 0, 0)  # 각도를 원하는 값으로 설정
            goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])  # 4개의 값을 각각 x, y, z, w로 전달

            goal_poses.append(goal_pose)
        return goal_poses

    def run_navigation(self):
        # Publish the entire path for visualization in RViz
        self.publish_path()

        # Start navigation
        self.navigator.waitUntilNav2Active()

        # Navigate to each waypoint in the fixed route
        for i, goal_pose in enumerate(self.goal_poses):
            # Send the robot to the current goal pose
            self.navigator.goToPose(goal_pose)

            # Wait until the robot reaches the goal or an obstacle is detected
            while not self.navigator.isTaskComplete():
                if self.obstacle_detected:
                    self.get_logger().info('장애물 감지! 이동 중단.')
                    while self.obstacle_detected:
                        rclpy.spin_once(self)
                    self.get_logger().info('장애물이 사라졌습니다. 이동 재개.')
                    self.navigator.goToPose(goal_pose)  # Resume navigation to the current goal
                    break

        # Notify once all waypoints have been reached
        self.get_logger().info('현동홀에 로봇이 도착했습니다.')

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationWithObstacle()
    try:
        navigation_node.run_navigation()
    except KeyboardInterrupt:
        navigation_node.get_logger().info('프로그램 종료 중...')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
