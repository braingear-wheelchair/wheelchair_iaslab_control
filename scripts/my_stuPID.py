#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry
from math import copysign

RATE = 50
MAX_DX = 0.5
MAX_DZ = 0.5

class PID:
    def __init__(self, p,d,i):
        self.p = 1
        self.d = 0
        self.i = 0
        self.prev_err = 0
        self.integral_err = 0

class my_stuPID:
    def __init__(self):
        rospy.init_node("vPID")
        self.cmd_vel = Twist()
        self.cmd_odom = Twist()
        self.err = Twist()
        self.rate = rospy.Rate(RATE)
        self.ptime = time.time()
        self.pid_x = PID(10.0,0.01,0.001)
        self.pid_z = PID(1.0,0.0,0.0)
        self.dt = 1 / RATE
        self.is_zero = True
        

    def setup(self):
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback )
        rospy.Subscriber('/cmd_vel_raw', Twist, self.vel_callback )
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    def publish_vel(self):
        if (self.is_zero):
            self.err = Twist()

        self.pub.publish(self.err)

    def odom_callback(self,msg):
        self.cmd_odom = msg.twist.twist
    
    def vel_callback(self,msg):
        if (abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001):
            self.is_zero = True
        else:
            self.is_zero = False
        self.ptime = time.time()
        self.cmd_vel = msg

    def compute_pid(self):
        dx = self.cmd_vel.linear.x - self.cmd_odom.linear.x
        dz = self.cmd_vel.angular.z - self.cmd_odom.angular.z

        self.pid_x.integral_err += dx * self.dt
        self.pid_z.integral_err += dz * self.dt

        d_ex = (dx - self.pid_x.prev_err) / self.dt
        d_ez = (dz - self.pid_z.prev_err) / self.dt

        self.pid_x.prev_err = dx
        self.pid_z.prev_err = dz

        pid_x = self.pid_x.p * dx + self.pid_x.i * self.pid_x.integral_err + self.pid_x.d * d_ex
        pid_z = self.pid_z.p * dz + self.pid_z.i * self.pid_z.integral_err + self.pid_z.d * d_ez

        if (abs(pid_x) > MAX_DX):
            pid_x = copysign(MAX_DX, pid_x)

        if (abs(pid_z) > MAX_DZ):
            pid_z = copysign(MAX_DX, pid_z)

        self.err.linear.x = self.cmd_vel.linear.x + pid_x
        self.err.angular.z = self.cmd_vel.angular.z + pid_z

    def run(self):
        while not rospy.is_shutdown():
            ctime = time.time()
            if ((ctime - self.ptime)*1000 < 40 ):
                self.compute_pid()
            else:
                self.err = Twist()
            self.publish_vel()
            self.rate.sleep()


def main():
    pid = my_stuPID()
    pid.setup()
    pid.run()

if __name__ == "__main__":
    main()
