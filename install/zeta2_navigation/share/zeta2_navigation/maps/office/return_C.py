#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from PIL import Image
import yaml

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
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)  # 경로를 RViz에 시각화할 토픽 생성
        self.obstacle_detected = False

        # PGM 파일과 YAML 파일에서 맵 데이터를 로드
        pgm_file = 'map1.pgm'
        yaml_file = 'mapping.yaml'
        self.width, self.height, self.resolution, self.origin = load_map_data(pgm_file, yaml_file)

    def publish_path(self, waypoints):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Path 메시지에 모든 웨이포인트를 추가
        for pose in waypoints:
            path_msg.poses.append(pose)

        # 경로 토픽에 경로 메시지 발행
        self.path_pub.publish(path_msg)

    def lidar_callback(self, scan_data):
        min_distance = min(scan_data.ranges)
        safe_distance = 0.5  # 안전 거리 설정
        if min_distance < safe_distance:
            self.obstacle_detected = True
            self.navigator.cancelTask()  # 네비게이션 작업 취소
            self.get_logger().info('장애물 감지! 로봇 멈춤.')
        else:
            self.obstacle_detected = False

    def run_navigation(self):
        goal_poses = []

        # GIMP에서 얻은 픽셀 좌표 (예시)
        pixel_coords = [
        (186, 54),   # Oseok Hall
        (186, 47), 
        (135, 47), 
        (135, 79),  # Initial Point
        ]

        # 픽셀 좌표를 월드 좌표로 변환하여 목표 지점 리스트 생성
        for u, v in pixel_coords:
            x, y = pixel_to_world(u, v, self.width, self.height, self.resolution, self.origin)

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0
            goal_poses.append(goal_pose)

        # 전체 경로를 RViz에 표시
        self.publish_path(goal_poses)

        # 네비게이션 시작
        self.navigator.waitUntilNav2Active()

        # 각 웨이포인트로 개별적으로 이동
        for i, goal_pose in enumerate(goal_poses):
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                if self.obstacle_detected:
                    self.get_logger().info('장애물 감지! 이동 중단.')
                    while self.obstacle_detected:
                        rclpy.spin_once(self)
                    self.get_logger().info('장애물이 사라졌습니다. 이동 재개.')
                    self.navigator.goToPose(goal_pose)  # 원래 웨이포인트로 다시 이동
                    break

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'{i + 1}번째 좌표 도착!')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('작업 취소됨.')
                return
            elif result == TaskResult.FAILED:
                self.get_logger().info('작업 실패.')
                return

        self.get_logger().info('모든 좌표를 성공적으로 방문했습니다.')

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
