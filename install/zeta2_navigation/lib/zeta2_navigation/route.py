import os
from rclpy.node import Node  # Node 클래스를 가져옴
from rclpy import rclpy
from rclpy.qos import QoSProfile
from nav2_simple_commander.robot_navigator import BasicNavigator
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler


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

        self.routes = {
            "route0":[(135, 79)],
            "route1": [(135, 79), (147, 79), (146, 65)],  # 식당
            "route2": [(146, 65), (166, 65), (160, 85)],  # 현동
            "route3": [(146, 65), (166, 65), (160, 85), (173, 84)],  # 느헤미야
            "route4": [(146, 65), (166, 65), (160, 85), (173, 84), (171, 54), (186, 54)],  # 오석관
            "route5": [(160, 85), (169, 85), (171, 109), (136, 106), (135, 79)],  # 복귀_현동
            "route6": [(173, 84), (203, 84), (204, 51), (164, 53), (164, 79), (135, 79)],  # 복귀_느헤미야
            "route7": [(186, 54), (186, 47), (135, 47), (135, 79)]  # 복귀_오석관
        }
        self.current_route = "route0"  # 기본 경로 설정
        self.set_route(self.current_route)  # 기본 경로 설정

    def set_route(self, route_name):
        if route_name in self.routes:
            self.fixed_route = self.routes[route_name]
            self.goal_poses = self.get_goal_poses()  # 새로운 경로로 목표 지점을 재계산
            self.get_logger().info(f'경로 변경: {route_name}')
            self.publish_path()  # 경로 변경 시 경로 발행

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.goal_poses:  
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp.pose.position.x
            pose.pose.position.y = wp.pose.position.y
            pose.pose.orientation = wp.pose.orientation
            path.poses.append(pose)
        
        self.path_pub.publish(path)
        self.get_logger().info('전체 경로 발행됨.')

    def lidar_callback(self, scan_data):
        min_distance = min(scan_data.ranges)
        safe_distance = 0.5  
        if min_distance < safe_distance:
            self.obstacle_detected = True
            self.navigator.cancelTask()  
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
            q = quaternion_from_euler(0, 0, 0)  
            goal_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            goal_poses.append(goal_pose)
        return goal_poses

    def run_navigation(self):
        self.publish_path()  # Publish the entire path for visualization in RViz
        self.navigator.waitUntilNav2Active()

        for i, goal_pose in enumerate(self.goal_poses):
            self.navigator.goToPose(goal_pose)
            while not self.navigator.isTaskComplete():
                if self.obstacle_detected:
                    self.get_logger().info('장애물 감지! 이동 중단.')
                    while self.obstacle_detected:
                        rclpy.spin_once(self)
                    self.get_logger().info('장애물이 사라졌습니다. 이동 재개.')
                    self.navigator.goToPose(goal_pose)  
                    break

        self.get_logger().info('식당에 로봇이 도착했습니다.')

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
