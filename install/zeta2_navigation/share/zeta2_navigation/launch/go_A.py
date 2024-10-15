#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node

class NavigationWithObstacle(Node):
    def __init__(self):
        super().__init__('navigation_with_obstacle')
        self.navigator = BasicNavigator()
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.obstacle_detected = False

    def lidar_callback(self, scan_data):
        min_distance = min(scan_data.ranges)
        safe_distance = 0.5
        if min_distance < safe_distance:
            self.obstacle_detected = True
            self.navigator.cancelTask()
            self.get_logger().info('장애물 감지! 로봇 멈춤.')
        else:
            self.obstacle_detected = False

    def run_navigation(self):
        goal_poses = []

        goal_pose1 = PoseStamped()                  #현동홀 좌표
        goal_pose1.header.frame_id = 'map'
        goal_pose1.pose.position.x = 0.64
        goal_pose1.pose.position.y = 0.04
        goal_pose1.pose.orientation.w = 1.0
        goal_poses.append(goal_pose1)

        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.pose.position.x = 0.64
        goal_pose2.pose.position.y = 0.49
        goal_pose2.pose.orientation.w = 1.0
        goal_poses.append(goal_pose2)

        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose3.pose.position.x = 1.439
        goal_pose3.pose.position.y = 0.49
        goal_pose3.pose.orientation.w = 1.0
        goal_poses.append(goal_pose3)

        goal_pose4 = PoseStamped()                  #최초 위치
        goal_pose4.header.frame_id = 'map'
        goal_pose4.pose.position.x = 1.39
        goal_pose4.pose.position.y = -0.809
        goal_pose4.pose.orientation.w = 1.0
        goal_poses.append(goal_pose4)

        self.navigator.waitUntilNav2Active()
        self.navigator.goThroughPoses(goal_poses)

        while not self.navigator.isTaskComplete():
            if not self.obstacle_detected:
                self.get_logger().info('경로 진행 중...')
            if self.obstacle_detected:
                self.get_logger().info('장애물 대기 중...')
                while self.obstacle_detected:
                    rclpy.spin_once(self)
                self.get_logger().info('장애물이 사라졌습니다. 경로 재개.')
                self.navigator.goThroughPoses(goal_poses)

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
