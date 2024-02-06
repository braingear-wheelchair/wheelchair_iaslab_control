#!/usr/bin/env python3
import rospy
import numpy
import random
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64
from random import *
import time
from rosneuro_msgs.msg import NeuroEvent

# Command codes
global FORWARD, LEFT, RIGHT, STOP, ERRORMASK
FORWARD       = 102
LEFT          = 101
RIGHT         = 103
STOP          = 100
ERRORMASK     = 5000
NORELEASEMASK = 4000

# flag
global flag_turn, flag_straight, flagError, flagGeneration
flag_turn = 0
flag_straight = 0
flagError = 0
flagGeneration = 0

# Timings
#global REFRACTORY_PERIOD 
#REFRACTORY_PERIOD = 2

global t_cmd, t_ref
t_cmd = 1.5
t_ref = 1.5

# Velocities
global VELX, VELZ
VELX = 0.05
VELZ = 0.4

# Global variables
global lastcmd, lastcmdtime, cmdvel
lastcmd = -1

global eventpub, event
event = NeuroEvent()

# Errors
global numberError, durExperiment, timeRelError, releasedError
durExperiment = 240 # seconds
releasedError = 0


def on_received_joy(Joy):
    global lastcmd, evtpub, flag_turn, flag_straight

    buttons = numpy.array(Joy.buttons)

    # This is to avoid sending commands when 2 buttons are pressed or when only
    # the joystick is moved
    if numpy.sum(buttons) == 1:
        index = numpy.nonzero(buttons)
        cmd = joy2cmd(index)

        # Questo serve per non pubblicare eventi quando il comando è lo stesso
        # del precedente
        #if lastcmd != cmd:
        set_command(cmd)
            #lastcmd = cmd

def joy2cmd(index):
    global FORWARD, LEFT, RIGHT, STOP
    
    command = -1

    if index[0] == 1:
        command = FORWARD
    elif index[0] == 2:
        command = STOP
    elif index[0] == 4:
        command = LEFT
    elif index[0] == 5:
        command = RIGHT

    return command


def set_neuroevent(cmd):
    global event
    event.event = cmd
    event.header.stamp = rospy.get_rostime()


def set_velocity(velx, velz):
    global cmdvel
    
    if velx != None:
        cmdvel.linear.x  = velx
    if velz != None:
        cmdvel.angular.z = velz


def set_command(cmd):
    global ERROR_THRESHOLD, lastcmd, lastcmdtime
    global event, evtpub, cmdvel, flag_turn, flag_straight, releasedError, tmp, flagError, flagGeneration, timeRelError
    global old_time

    # Check if it is within the refractory period
    #elapsed = (rospy.Time.now() - lastcmdtime).to_sec()

    # If the command is stop, it publishes the event and cmdvel and return
    if cmd == STOP:
        set_neuroevent(cmd)
        evtpub.publish(event)
        set_velocity(0.0, 0.0)
        flag_turn = 0
        flag_straight = 0
        return None


    elif flag_turn == 1 or flag_straight == 1:
        old_time = rospy.Time.now()
        set_neuroevent(cmd + NORELEASEMASK)
        evtpub.publish(event)
        return None
    
    
    elif flag_turn == 0 and flag_straight == 0:   
    
    	lastcmdtime = rospy.Time.now()
    	# If the command is forward, it publishes the event and cmdvel and return
    	if cmd == FORWARD:
        	set_neuroevent(cmd)
        	evtpub.publish(event)
        	set_velocity(VELX, 0.0)
        	return None

    	# Otherwise it checks if it will be an error
    	
        
    	else: 
            print("[error_controller_discrete] - Correct command")
            old_time = rospy.Time.now()
            set_neuroevent(cmd)
            evtpub.publish(event)
            if cmd == LEFT:
                flag_turn = 1
                set_velocity(None, VELZ)
            elif cmd == RIGHT:
                flag_turn = 1
                set_velocity(None, -VELZ)

def second_joy_callback(data):
    buttons = numpy.array(data.buttons)
    if buttons[0] == 1:
        global new_event, delay, old_time
        new_time = rospy.Time.now()
        delay = (new_time - old_time).to_sec()
        new_event = True


if __name__ == '__main__':
    
    global evtpub, cmdvel, numberError, timeWindow, initialtime
    
    cmdvel = Twist()

  
    rospy.init_node("last_version_discrete2_noError")
    initialtime = rospy.Time.now()

    cmdpub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    evtpub = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1)

    delaypub = rospy.Publisher("/delay", Float64, queue_size=1)

    rospy.Subscriber("/j0", Joy, on_received_joy, queue_size=1)
    rospy.Subscriber("/j1", Joy, second_joy_callback, queue_size=1)

    rate = rospy.Rate(500) 

    #lastcmdtime = rospy.Time.now()
    global delay, new_event
    new_event = False
    global old_time
    old_time = rospy.Time.now()

    msg_delay = Float64()

    while not rospy.is_shutdown():

        if flag_turn == 1:
            #publish cmdvel
            if (rospy.Time.now() - lastcmdtime).to_sec() > t_cmd:
                set_velocity(VELX, 0.0)
                flag_turn = 0
                flag_straight = 1
                lastcmdtime = rospy.Time.now()

        cmdpub.publish(cmdvel) 
                       
        if flag_straight == 1:
            if (rospy.Time.now() - lastcmdtime).to_sec() > t_ref:
                flag_straight = 0

        if new_event:
            msg_delay = delay
            delaypub.publish(msg_delay)
            new_event = False


        rate.sleep()    

