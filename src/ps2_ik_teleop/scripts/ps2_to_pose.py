#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
import math

class PS2TeleopIK:
    def __init__(self):
        rospy.init_node("ps2_to_pose")

        self.pub = rospy.Publisher("/ik_target_pose", PoseStamped, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        # initial pose (safe position)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "base"
        self.pose.pose.position.x = 0.6
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.3

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.step_pos = 0.01
        self.step_rot = 0.03

    def joy_cb(self, msg):
        ax = msg.axes
        btn = msg.buttons

        # --- DEADZONE ---
        def dz(v): return 0.0 if abs(v) < 0.1 else v

        lx = dz(ax[0])
        ly = dz(ax[1])
        rx = dz(ax[2])
        ry = dz(ax[3])

        # --- POSITION CONTROL ---
        self.pose.pose.position.x += self.step_pos * ly
        self.pose.pose.position.y += self.step_pos * lx
        self.pose.pose.position.z += self.step_pos * ry

        # --- ORIENTATION CONTROL ---
        # L1 / R1 for yaw
        if btn[4]: self.yaw += self.step_rot
        if btn[5]: self.yaw -= self.step_rot

        # L2 / R2 for pitch
        if len(btn) > 6:
            if btn[6]: self.pitch += self.step_rot
            if btn[7]: self.pitch -= self.step_rot

        # simple Euler → quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(
            self.roll, self.pitch, self.yaw
        )

        self.pose.pose.orientation.x = qx
        self.pose.pose.orientation.y = qy
        self.pose.pose.orientation.z = qz
        self.pose.pose.orientation.w = qw

        self.pose.header.stamp = rospy.Time.now()
        self.pub.publish(self.pose)

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
    PS2TeleopIK()
    rospy.spin()
