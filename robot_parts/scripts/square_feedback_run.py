#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def yaw_from_quat(q):
    # q is geometry_msgs/Quaternion
    # yaw = atan2(2(w*z + x*y), 1 - 2(y*y + z*z))
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def ang_diff(a, b):
    # shortest difference a-b in [-pi, pi]
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d

class DriveSquareOdom(Node):
    def __init__(self):
        super().__init__('drive_square_odom')
        self.cmd = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        # params
        self.side_len = 1.0    # meters
        self.vx = 2.0         # m/s
        self.turn_rate = 1.6    # rad/s (commanded)
        self.turn_angle = math.pi / 2.0  # 90 deg
        self.pause = 0.15       # s

        self.state = 'WAIT_ODOM'
        self.pose = None
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.sides_done = 0

        self.dt = 1.0/30.0
        self.timer = self.create_timer(self.dt, self.loop)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def loop(self):

        if self.pose is None:
            return  # wait for odom
        x, y, yaw = self.pose

        if self.state == 'WAIT_ODOM':
            self.start_x, self.start_y, self.start_yaw = x, y, yaw
            self.state = 'FORWARD'
            return

        if self.state == 'FORWARD':
            # command forward
            t = Twist(); t.linear.x = self.vx
            self.cmd.publish(t)
            # distance traveled
            dx = x - self.start_x; dy = y - self.start_y
            if math.hypot(dx, dy) >= self.side_len:
                self.cmd.publish(Twist())  # stop briefly
                time.sleep(self.pause)
                # setup for turn
                self.start_yaw = yaw
                self.state = 'TURN'
            return

        if self.state == 'TURN':
            # command positive yaw
            t = Twist(); t.angular.z = self.turn_rate
            self.cmd.publish(t)
            if abs(ang_diff(yaw, self.start_yaw)) >= self.turn_angle:
                self.cmd.publish(Twist())
                time.sleep(self.pause)
                # next side
                self.start_x, self.start_y = x, y
                self.sides_done += 1
                # if self.sides_done >= 4:  %toggle this command to make it turn for only a square
                if not rclpy.ok():
                    self.get_logger().info('Square complete.')
                    self.cmd.publish(Twist())
                    rclpy.shutdown()
                    return
                self.state = 'FORWARD'
            return

def main():
    rclpy.init()
    node = DriveSquareOdom()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
