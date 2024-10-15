import sys
import subprocess
import time
import rclpy
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLCDNumber, QRadioButton, QSlider, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QLabel, QGraphicsEllipseItem
from PyQt5.uic import loadUi
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from rclpy.node import Node
from PyQt5.QtGui import QPixmap, QColor, QPainter

class CmdVelSubscriber(Node):
    def __init__(self, signal, position_signal):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.position_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.position_callback,
            10
        )
        self.signal = signal
        self.position_signal = position_signal

    def cmd_vel_callback(self, msg):
        self.signal.emit(msg.linear.x, msg.angular.z)  # Emit both linear and angular velocities

    def position_callback(self, msg):
        # Extract the position from the PoseWithCovarianceStamped message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.position_signal.emit(x, y)  # Emit x and y positions

class RosThread(QThread):
    update_signal = pyqtSignal(float, float)  # Updated to emit both linear and angular velocities
    position_signal = pyqtSignal(float, float)  # Updated to emit x and y positions

    def __init__(self):
        super().__init__()
        self.node = CmdVelSubscriber(self.update_signal, self.position_signal)

    def run(self):
        rclpy.spin(self.node)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()


class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        loadUi("robot_gui.ui", self)  # Load UI file created by Qt Designer


        # QGraphicsView widget setup
        self.graphics_view = self.findChild(QGraphicsView, 'graphicsView')
        self.scene = QGraphicsScene()
        self.graphics_view.setScene(self.scene)

        # Load map image
        map_path = '/home/jinyoung/zeta_ws/src/zeta2_edu_autonomous/zeta2_navigation/maps/school/slam_toolbox_handong3.pgm'
        pixmap = QPixmap(map_path)
        if pixmap.isNull():
            print(f"Failed to load map image from {map_path}")
        else:
            pixmap_item = QGraphicsPixmapItem(pixmap)
            self.scene.addItem(pixmap_item)

        # Initialize robot position indicator
        self.robot_marker = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.robot_marker.setBrush(QColor('red'))
        self.scene.addItem(self.robot_marker)


        # ROS2 node and thread setup
        rclpy.init()  # Initialize ROS 2
        self.ros_thread = RosThread()
        self.ros_thread.update_signal.connect(self.update_velocity_lcd)
        self.ros_thread.update_signal.connect(self.update_angular_lcd)
        self.ros_thread.start()

        # Current running processes
        self.bringup_process = None
        self.navigation_process = None

        # Buttons setup
        self.bringup_button = self.findChild(QPushButton, 'pushButton_bringup')
        self.bringup_button.clicked.connect(self.toggle_bringup)

        self.navigation_button = self.findChild(QPushButton, 'pushButton_nav')
        self.navigation_button.clicked.connect(self.toggle_navigation)

        # QLCDNumber setup
        self.lcd_display = self.findChild(QLCDNumber, 'lcdNumber')
        self.lcd_display_2 = self.findChild(QLCDNumber, 'lcdNumber_2')

        # Radio button and control buttons setup
        self.manual_button = self.findChild(QRadioButton, 'radioButton')
        self.manual_button.toggled.connect(self.manual_control)

        self.forward_button = self.findChild(QPushButton, 'pushButton_foward')
        self.forward_button.pressed.connect(self.move_forward_start)
        self.forward_button.released.connect(self.stop_movement)

        self.back_button = self.findChild(QPushButton, 'pushButton_back')
        self.back_button.pressed.connect(self.move_back_start)
        self.back_button.released.connect(self.stop_movement)

        self.ccw_button = self.findChild(QPushButton, 'pushButton_ccw')
        self.ccw_button.pressed.connect(self.turn_ccw_start)
        self.ccw_button.released.connect(self.stop_movement)

        self.cw_button = self.findChild(QPushButton, 'pushButton_cw')
        self.cw_button.pressed.connect(self.turn_cw_start)
        self.cw_button.released.connect(self.stop_movement)

        # Slider setup
        self.velocity_slider = self.findChild(QSlider, 'verticalSlider')
        self.angular_slider = self.findChild(QSlider, 'verticalSlider_2')

        self.velocity_slider.valueChanged.connect(self.update_velocity)
        self.angular_slider.valueChanged.connect(self.update_angular)

        # Position labels setup
        self.label_x = self.findChild(QLabel, 'label_x')
        self.label_y = self.findChild(QLabel, 'label_y')

        # Initialize slider values
        self.velocity_slider.setRange(0, 100)  # 0-100 range
        self.angular_slider.setRange(0, 100)   # 0-100 range
        self.velocity_slider.setValue(100)
        self.angular_slider.setValue(100)

        # ROS publisher setup
        self.node = rclpy.create_node('robot_gui')  # Create ROS node
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)  # Publisher setup

        # Timer for publishing messages
        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_cmd_vel)
        self.timer.start(100)  # Timer interval in milliseconds

        self.current_cmd = Twist()  # Store the current command

        self.reset_button = self.findChild(QPushButton, 'pushButton_reset')
        self.reset_button.clicked.connect(self.reset_robot)

    def reset_robot(self): 
        command = 'sshpass -p 1 ssh zeta@192.168.1.77 "reboot"'
        print("Robot Reboot")

    def update_velocity_lcd(self, linear_x, _):
        self.lcd_display.display(linear_x)

    def update_angular_lcd(self, _, angular_z):
        self.lcd_display_2.display(angular_z)

    def update_position_and_marker(self, x, y):
        self.label_x.setText(f"X: {x:.2f}")
        self.label_y.setText(f"Y: {y:.2f}")

        # Update robot position marker
        self.robot_marker.setPos(x, y)

        # Adjust view to keep the robot marker visible
        self.graphics_view.centerOn(self.robot_marker)

    def toggle_bringup_robot1(self):
        if self.bringup_process is not None:
            self.bringup_process.terminate()
            self.bringup_process.wait()
            self.bringup_process = None
            print("Robot1 bringup.launch.py process end.")
        else:
            command = 'sshpass -p 1 ssh zeta@192.168.1.77 "cd ~/zeta_ws; source install/setup.bash; export ROS_DOMAIN_ID=11; ros2 launch zeta2_bringup zeta2_bringup.launch.py"'
            self.bringup_process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])
            time.sleep(7)
            self.navigation_button.setEnabled(True)
            print("Robot1 bringup.launch.py process start.")

    def toggle_bringup_robot2(self):
        if self.bringup_process is not None:
            self.bringup_process.terminate()
            self.bringup_process.wait()
            self.bringup_process = None
            print("Robot2 bringup.launch.py process end.")
        else:
            command = 'sshpass -p 1 ssh zeta@192.168.1.77 "cd ~/zeta_ws; source install/setup.bash; export ROS_DOMAIN_ID=11; ros2 launch zeta2_bringup zeta2_bringup.launch.py"'
            self.bringup_process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])
            time.sleep(7)
            self.navigation_button.setEnabled(True)
            print("Robot2 bringup.launch.py process start.")

    def toggle_navigation(self):
        if self.navigation_process is not None and self.navigation_process.poll() is None:
            self.navigation_process.terminate()
            self.navigation_process.wait()
            self.navigation_process = None
            print("navigation.launch.py process end.")
        else:
            command = "cd ~/zeta_ws && source install/setup.bash && export ROS_DOMAIN_ID=11 && ros2 launch zeta2_navigation zeta2_navigation.launch.py"
            self.navigation_process = subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'{command}; exec bash'])
            print("navigation.launch.py process start.")

    def manual_control(self):
        # Toggle buttons based on the radio button state
        is_checked = self.manual_button.isChecked()
        self.forward_button.setEnabled(is_checked)
        self.back_button.setEnabled(is_checked)
        self.ccw_button.setEnabled(is_checked)
        self.cw_button.setEnabled(is_checked)

    def update_velocity(self):
        # Update current_cmd but don't publish yet
        self.current_cmd.linear.x = self.velocity_slider.value() / 100.0  # Convert 0-100 range to 0.0-1.0 range

    def update_angular(self):
        # Update current_cmd but don't publish yet
        self.current_cmd.angular.z = self.angular_slider.value() / 100.0  # Convert 0-100 range to 0.0-1.0 range

    def move_forward_start(self):
        # Move forward using the slider value
        self.current_cmd.linear.x = self.velocity_slider.value() / 100.0
        self.current_cmd.angular.z = 0.0
        self.update_cmd_vel()

    def move_back_start(self):
        # Move backward using the slider value
        self.current_cmd.linear.x = -self.velocity_slider.value() / 100.0
        self.current_cmd.angular.z = 0.0
        self.update_cmd_vel()

    def turn_ccw_start(self):
        # Turn counterclockwise using the slider value
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = self.angular_slider.value() / 100.0
        self.update_cmd_vel()

    def turn_cw_start(self):
        # Turn clockwise using the slider value
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = -self.angular_slider.value() / 100.0
        self.update_cmd_vel()

    def stop_movement(self):
        # Stop all movement
        self.current_cmd.linear.x = 0.0
        self.current_cmd.angular.z = 0.0
        self.update_cmd_vel()

    def update_cmd_vel(self):
        # Publish the current command
        self.publisher.publish(self.current_cmd)

    def publish_cmd_vel(self):
        # Ensure the latest command is published
        self.update_cmd_vel()

    def closeEvent(self, event):
        self.ros_thread.stop()
        self.ros_thread.wait()
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())
