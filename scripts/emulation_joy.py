#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist 
from math import copysign
from numpy import sign

# TODO: move this to the 
RATE = 150
MAX_DX = 1e-4
MAX_DZ = 1e-4


class emulation_joy:
    def __init__(self):
        rospy.init_node("joy_emulator")
        self.prev_cmd = Twist()
        self.required_cmd = Twist()
        self.required_new_cmd = False

    def setup(self):
        rospy.Subscriber('/input_cmd_vel', Twist, self.vel_callback)
        self.pub = rospy.Publisher('/corrected_cmd_vel', Twist, queue_size=1)

    def publish_vel(self):
        self.pub.publish(self.required_cmd)

    def vel_callback(self, msg: Twist):
        self.required_cmd = msg
        self.required_new_cmd = True

    def compute_cmd(self):
        if (abs(self.required_cmd.linear.x - self.prev_cmd.linear.x) > MAX_DX):
            tmp = sign(self.required_cmd.linear.x - self.prev_cmd.linear.x)
            self.prev_cmd.linear.x = self.prev_cmd.linear.x + tmp * MAX_DX
        else:
            self.prev_cmd.linear.x = self.required_cmd.linear.x

        if (abs(self.required_cmd.angular.z - self.prev_cmd.angular.z) > MAX_DZ):
            tmp = sign(self.required_cmd.angular.z - self.prev_cmd.angular.z)
            self.prev_cmd.angular.z = self.prev_cmd.angular.z + tmp * MAX_DZ
        else:
            self.prev_cmd.angular.z = self.required_cmd.angular.z

    def run(self):
        self.rate = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            if (self.required_new_cmd):
                self.compute_cmd()
                self.publish_vel()
                self.required_new_cmd = False
            self.rate.sleep()


def main():
    dev = emulation_joy()
    dev.setup()
    dev.run()

if __name__ == "__main__":
    main()
