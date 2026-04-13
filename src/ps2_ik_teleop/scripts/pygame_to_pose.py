#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import pygame
import math

class PygameTeleop:
    def __init__(self):
        rospy.init_node("pygame_ps2_teleop")

        self.pub = rospy.Publisher("/ik_target_pose", PoseStamped, queue_size=1)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise Exception("No joystick found")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()

        self.pose = PoseStamped()
        self.pose.header.frame_id = "base"
        self.pose.pose.position.x = 0.6
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.3

        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.step = 0.005

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            pygame.event.pump()

            # read axes
            ax0 = self.js.get_axis(0)
            ax1 = self.js.get_axis(1)
            ax2 = self.js.get_axis(2)
            ax3 = self.js.get_axis(3)

            # deadzone
            def dz(v): return 0 if abs(v) < 0.1 else v

            ax0, ax1, ax2, ax3 = dz(ax0), dz(ax1), dz(ax2), dz(ax3)

            # position control
            self.pose.pose.position.x += self.step * ax1
            self.pose.pose.position.y += self.step * ax0
            self.pose.pose.position.z += self.step * ax3

            # simple orientation
            if self.js.get_button(4):
                self.yaw += 0.02
            if self.js.get_button(5):
                self.yaw -= 0.02

            qx, qy, qz, qw = self.euler_to_quaternion(
                self.roll, self.pitch, self.yaw
            )

            self.pose.pose.orientation.x = qx
            self.pose.pose.orientation.y = qy
            self.pose.pose.orientation.z = qz
            self.pose.pose.orientation.w = qw

            self.pose.header.stamp = rospy.Time.now()
            self.pub.publish(self.pose)

            rate.sleep()

    def euler_to_quaternion(self, r, p, y):
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

if __name__ == "__main__":
    node = PygameTeleop()
    node.run()
