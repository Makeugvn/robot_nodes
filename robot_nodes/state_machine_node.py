#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
import time

class RobotState:
    MAPPING   = 'mapping'
    NAVIGATING = 'navigating'
    IDLE      = 'idle'

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.state = RobotState.IDLE
        self.map_ready = False

        # Publisher untuk notify state ke node lain
        self.state_pub = self.create_publisher(
            String, '/robot_state', 10)

        # Publisher untuk trigger scan (ke servo controller nanti)
        self.scan_trigger_pub = self.create_publisher(
            Bool, '/trigger_scan', 10)

        # Publisher stop robot
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Subscribe scan selesai (dari servo controller nanti)
        self.scan_done_sub = self.create_subscription(
            Bool, '/scan_complete', self.scan_done_cb, 10)

        # Subscribe goal dari RViz2 (2D Goal Pose)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Nav2 action client
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Timer status log
        self.create_timer(2.0, self.log_state)

        self.get_logger().info('State machine ready. State: IDLE')
        self.get_logger().info('Publish ke /trigger_scan=True untuk mulai mapping')

    # ── Callbacks ────────────────────────────────────────────

    def scan_done_cb(self, msg):
        """Dipanggil saat servo selesai sweep 180°"""
        if msg.data and self.state == RobotState.MAPPING:
            self.get_logger().info('Scan selesai! Peta diperbarui.')
            self.map_ready = True
            self.set_state(RobotState.IDLE)

    def goal_cb(self, msg):
        """Dipanggil saat user klik 2D Goal Pose di RViz2"""
        if self.state == RobotState.NAVIGATING:
            self.get_logger().warn('Sudah navigasi, goal baru diabaikan')
            return
        if not self.map_ready:
            self.get_logger().warn('Peta belum siap! Lakukan mapping dulu.')
            return

        self.get_logger().info(
            f'Goal diterima: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.set_state(RobotState.NAVIGATING)
        self.send_nav_goal(msg)

    # ── State transitions ─────────────────────────────────────

    def set_state(self, new_state):
        self.state = new_state
        msg = String()
        msg.data = new_state
        self.state_pub.publish(msg)
        self.get_logger().info(f'>>> State: {new_state.upper()}')

    def start_mapping(self):
        """Panggil ini untuk mulai fase mapping"""
        if self.state == RobotState.NAVIGATING:
            self.get_logger().warn('Sedang navigasi, selesaikan dulu')
            return

        self.stop_robot()
        self.set_state(RobotState.MAPPING)
        self.map_ready = False

        # Trigger servo sweep (untuk hardware nanti)
        # Di simulasi, scan langsung jalan karena lidar selalu aktif
        trigger = Bool()
        trigger.data = True
        self.scan_trigger_pub.publish(trigger)
        self.get_logger().info('Trigger scan dikirim ke servo controller')

        # Di simulasi: anggap scan selesai setelah 3 detik
        # (nanti diganti dengan /scan_complete dari servo controller)
        self.create_timer(3.0, self._sim_scan_done)

    def _sim_scan_done(self):
        """Simulasi scan selesai — di hardware diganti callback servo"""
        if self.state == RobotState.MAPPING:
            done = Bool()
            done.data = True
            self.scan_done_cb(done)

    def stop_robot(self):
        from geometry_msgs.msg import Twist
        self.cmd_vel_pub.publish(Twist())  # zero velocity

    # ── Nav2 ──────────────────────────────────────────────────

    def send_nav_goal(self, pose_stamped):
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 tidak tersedia!')
            self.set_state(RobotState.IDLE)
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.nav_feedback_cb)
        future.add_done_callback(self.nav_goal_response_cb)

    def nav_goal_response_cb(self, future):
        result = future.result()
        if not result.accepted:
            self.get_logger().error('Goal ditolak Nav2!')
            self.set_state(RobotState.IDLE)
            return
        self.get_logger().info('Goal diterima Nav2, robot bergerak...')
        result.get_result_async().add_done_callback(self.nav_result_cb)

    def nav_result_cb(self, future):
        self.get_logger().info('Navigasi selesai! Kembali ke IDLE.')
        self.set_state(RobotState.IDLE)
        # Otomatis mulai mapping lagi setelah sampai tujuan
        self.get_logger().info('Memulai mapping ulang...')
        self.start_mapping()

    def nav_feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().info(f'Sisa jarak: {dist:.2f}m', throttle_duration_sec=2.0)

    def log_state(self):
        self.get_logger().info(
            f'State: {self.state} | Map ready: {self.map_ready}',
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()

    # Mulai mapping otomatis saat node start
    node.start_mapping()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()