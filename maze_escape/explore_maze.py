"""미로 전체를 자율로 한 바퀴 돌면서 SLAM 지도를 완성시키는 노드.

미로 그래프(스패닝 트리)를 DFS로 완전탐색하는 셀 경로를 미리 계산해두고,
/odom 피드백을 보면서 회전->직진을 반복해 각 웨이포인트로 이동한다.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from maze_escape.maze_grid import cell_center, carve_maze, dfs_walk

LINEAR_SPEED = 0.18
ANGULAR_SPEED = 0.8
ANGLE_TOLERANCE = 0.05
DIST_TOLERANCE = 0.08

WIDTH = 10
HEIGHT = 10
SEED = 42


def yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class MazeExplorer(Node):
    def __init__(self, waypoints):
        super().__init__('maze_explorer')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.waypoints = waypoints
        self.wp_index = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.pose_received = False
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'전체 {len(waypoints)}개 웨이포인트 탐색 시작')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose_received = True

    def control_loop(self):
        if not self.pose_received:
            return
        if self.wp_index >= len(self.waypoints):
            self.publisher.publish(Twist())
            self.get_logger().info('탐색 완료')
            return

        tx, ty = self.waypoints[self.wp_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < DIST_TOLERANCE:
            self.wp_index += 1
            if self.wp_index % 10 == 0 or self.wp_index == len(self.waypoints):
                self.get_logger().info(f'진행: {self.wp_index}/{len(self.waypoints)}')
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(target_yaw - self.yaw)

        twist = Twist()
        if abs(yaw_err) > ANGLE_TOLERANCE:
            twist.angular.z = ANGULAR_SPEED if yaw_err > 0 else -ANGULAR_SPEED
        else:
            twist.linear.x = LINEAR_SPEED
            twist.angular.z = yaw_err * 1.5

        self.publisher.publish(twist)


def main():
    grid = carve_maze(WIDTH, HEIGHT, SEED)
    cell_path = dfs_walk(grid, start=(0, 0))
    waypoints = [cell_center(cell) for cell in cell_path]

    rclpy.init()
    node = MazeExplorer(waypoints)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.publisher.publish(Twist())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
