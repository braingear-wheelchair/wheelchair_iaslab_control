#!/usr/bin/env python3
import rospy
import numpy as np
import random as rand
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist 
import time
from rosneuro_msgs.msg import NeuroEvent

# Velocities
VELX = 0.1
VELZ = 0.9

RATE = 100

# TIMINGS
TIME_REFRAC_PERIOD = 2.5
TIME_ERROR = 1
TIME_TURN = 1

PROBABILITY_INCREASE = 0.003
MAX_PROBABILITY = 0.2
TIME_PROBABILITY_INCREASE = 2

class Codes:
    def __init__(self):
        self.FORWARD       = 102
        self.LEFT          = 101
        self.RIGHT         = 103
        self.STOP          = 100
        self.ERRORMASK     = 5000
        self.NORELEASEMASK = 4000
    
    def getCommand(self, index):
        if index == 0:
            return self.FORWARD
        elif index == 2:
            return self.STOP
        elif index == 4:
            return self.LEFT
        elif index == 5:
            return self.RIGHT
        return self.FORWARD

class Controller:
    def __init__(self):
        # Create the node
        rospy.init_node('controller')
        # Create the codes for the particular joystick
        self.codes = Codes()
        # Create the void messages
        self.event = NeuroEvent()
        self.cmdvel = Twist()
        # Create the rate
        self.rate = rospy.Rate(RATE)
        # The probability to generate an error
        self.probability = 0
        # Create the state (a boolean variable that indicates if it follows the requested trajectory or not)
        self.follow_order = True
        # Create the variable to indicate if it is in the refractory period
        self.refractory_period = False
        # Set the current command
        self.current_command = self.codes.STOP
        # Crate the booleans that say if it is needed to publish the messages
        self.pub_cmd = True
        self.pub_event = False
        # Create the MASK to add to the command -> default = 0
        self.event_mask = 0
        self.first_curve = True
        self.start_probability_increase = False

    def run(self):
        while not rospy.is_shutdown():

            if self.start_probability_increase:
                if (self.probability < MAX_PROBABILITY):
                    self.probability += PROBABILITY_INCREASE/RATE

            if self.pub_cmd:
                self.set_velocity()
                self.cmdpub.publish(self.cmdvel)

            if self.pub_event:
                self.set_neuroevent()
                self.evtpub.publish(self.event)
                self.pub_event = False

            self.rate.sleep()

    def end_error(self, data = None):
        self.follow_order = True
        if (not self.current_command == self.codes.STOP):
            self.current_command = self.codes.FORWARD
        self.event_mask = 0

    def end_turn(self, data = None):
        if (not self.current_command == self.codes.STOP):
                self.current_command = self.codes.FORWARD
        self.pub_event = True
            

    def end_refractory_period(self, data = None):
        self.refractory_period = False
        if (not self.current_command == self.codes.STOP):
            self.current_command = self.codes.FORWARD
        self.event_mask = 0

    def set_velocity(self):
        if self.current_command != self.codes.STOP:
            # set the linear velocity
            self.cmdvel.linear.x = VELX
            self.cmdvel.angular.z = 0.0
            
            # set the angular velocity if needed
            if self.current_command == self.codes.LEFT:
                self.cmdvel.angular.z = VELZ
            elif self.current_command == self.codes.RIGHT:
                self.cmdvel.angular.z = -VELZ
            
            # set the reverse velocity if needed
            if not self.follow_order:
                self.cmdvel.angular.z = - self.cmdvel.angular.z

        else:
            self.cmdvel.linear.x = 0.0
            self.cmdvel.angular.z = 0.0

    def set_neuroevent(self):
        self.event.event = self.current_command + self.event_mask
        self.event.header.stamp = rospy.get_rostime()
       

    def setup(self):
        self.setup_listeners()
        self.setup_publishers()

    def setup_listeners(self):
        rospy.Subscriber('/joy', Joy, self.on_received_joy)

    def setup_publishers(self):
        self.evtpub = rospy.Publisher('/events/bus', NeuroEvent, queue_size=1)
        self.cmdpub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def on_received_joy(self, Joy):
        buttons = np.array(Joy.buttons)
        # Check if there is only one button pressed
        if np.sum(buttons) == 1:
            index = np.nonzero(buttons)[0]
            self.current_command = self.codes.getCommand(index)
            # Check if the command need additional logic as turn, refractory period or error
            self.check_command(self.current_command)

    def check_command(self, command):
        # In any case request to publish the event
        self.pub_event = True

        # Do the logic only if the command is not stop
        if command != self.codes.STOP:        
            # First check if we are in the refractory period, if this is the case set the NO RELEASE
            # flag and return
            if self.refractory_period:
                # Set the NO RELEASE mask
                self.event_mask = self.codes.NORELEASEMASK

            # Check if we want to turn
            elif command == self.codes.LEFT or command == self.codes.RIGHT:

                if self.first_curve:
                    self.first_curve = False
                    self.start_probability_increase = True

                # If we want to turn determine if we want to follow the order or not
                if (rand.random() < self.probability):
                    self.follow_order = False
                    # Put a time callback to end the error
                    rospy.Timer(rospy.Duration(TIME_ERROR), self.end_error, oneshot=True)
                    self.event_mask = self.codes.ERRORMASK
                    self.probability = 0
                else:
                    self.event_mask = 0

                # Set the refractory period
                self.refractory_period = True

                # Put a time callback to remove the refractory period
                rospy.Timer(rospy.Duration(TIME_REFRAC_PERIOD), self.end_refractory_period, oneshot=True)

                # Put a time to end the curve
                rospy.Timer(rospy.Duration(TIME_TURN), self.end_turn, oneshot=True)

    
def main():
    controller = Controller()
    controller.setup()
    controller.run()

if __name__ == '__main__':
    main()

