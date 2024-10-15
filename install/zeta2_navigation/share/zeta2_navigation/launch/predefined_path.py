#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

class NavigationWithObstacle(Node):
    def __init__(self):
        super().__init__('navigation_with_obstacle')

        self.navigator = BasicNavigator()

        # LiDAR 데이터 구독
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.obstacle_detected = False  # 장애물 감지 플래그

    def lidar_callback(self, scan_data):
        # LiDAR 데이터를 분석하여 장애물이 감지되면 멈추기 위한 논리
        min_distance = min(scan_data.ranges)  # LiDAR에서 측정한 최소 거리
        safe_distance = 0.5  # 안전 거리 설정 (0.5m)

        if min_distance < safe_distance:
            self.obstacle_detected = True  # 장애물 감지
            self.navigator.cancelTask()  # 장애물 감지 시 이동 취소
            self.get_logger().info('장애물 감지! 로봇 멈춤.')
        else:
            self.obstacle_detected = False  # 장애물이 없을 경우

    def run_navigation(self):
        # 경로 설정
        goal_poses = []

        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.pose.position.x = 1.0
        goal_pose1.pose.position.y = 0.0
        goal_pose1.pose.orientation.w = 1.0
        goal_poses.append(goal_pose1)

        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.pose.position.x = 2.0
        goal_pose2.pose.position.y = 1.0
        goal_pose2.pose.orientation.w = 1.0
        goal_poses.append(goal_pose2)

        self.navigator.waitUntilNav2Active()

        # 경로 시작
        self.navigator.goThroughPoses(goal_poses)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()

            # 장애물이 없는 경우 계속해서 경로 진행
            if not self.obstacle_detected:
                self.get_logger().info('경로 진행 중...')

            # 장애물 감지 시 로봇이 멈춘 상태에서 대기, 장애물이 사라지면 재개
            if self.obstacle_detected:
                self.get_logger().info('장애물 대기 중...')
                while self.obstacle_detected:  # 장애물이 사라질 때까지 대기
                    rclpy.spin_once(self)
                self.get_logger().info('장애물이 사라졌습니다. 경로 재개.')
                self.navigator.goThroughPoses(goal_poses)  # 경로 다시 재개

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('목표 지점 도착!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('작업 취소됨.')
        elif result == TaskResult.FAILED:
            self.get_logger().info('작업 실패.')

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
