#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.pub=self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped',10)
        self.rate=self.create_rate(10)

    def send_cmd(self,vx,wz,duration):
        t_end=time.time() +duration
        msg=Twist()
        msg.linear.x=vx
        msg.angular.z=wz

        while rclpy.ok() and time.time() < t_end:
            self.pub.publish(msg)
            self.rate.sleep

        self.pub.publish(Twist())

def main():
    rclpy.init()
    node= DriveSquare()

    for _ in range(4):
        node.send_cmd(1.0,0.0,1.0)
        time.sleep(0.5)
        node.send_cmd(0.0,1.6,1.6)
        time.sleep(0.5)
    rclpy.shutdown()


if __name__=="__main__":
    main()


