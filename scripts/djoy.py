#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class Control:
    def __init__(self):
        rospy.init_node("djoy")
        self.cmd = Twist()
        self.pub = rospy.Publisher('djoy/cmd_vel', Twist, queue_size=1)
        self.e_stop = Bool()
        self.e_stop.data = False
        self.pub_e_stop = rospy.Publisher('/djoy/e_stop', Bool, queue_size=1)
        self.compensation = rospy.get_param('compensation', 0.3)
        self.have_cmd = False
        self.have_estop = False
        self.rate = rospy.get_param('rate', 50)

    def joy_callback(self, msg):
        self.cmd.linear.x = msg.axes[1] * self.compensation
        self.cmd.angular.z = msg.axes[0] * self.compensation
        self.have_cmd = True
        if (abs(msg.axes[0]) < 0.001 and abs(msg.axes[1]) < 0.001):
            self.have_cmd = False
        if (msg.buttons[1] == 1):
            self.e_stop.data = not self.e_stop.data # toggle True
            self.have_estop = True
        pass

    def publish_twist(self):
        self.pub.publish(self.cmd)

    def publish_e_stop(self):
        self.pub_e_stop.publish(self.e_stop)

    def setup(self):
        rospy.Subscriber('joy', Joy, self.joy_callback)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            if (self.have_cmd):
                self.publish_twist()
            if (self.have_estop):
                self.publish_e_stop()
                self.have_estop = False
            rate.sleep()

def main():
    c = Control()
    c.setup()
    c.run()

if __name__ == '__main__':
    main()


