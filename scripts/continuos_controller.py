#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist 
from random import *
from rosneuro_msgs.msg import NeuroEvent
from numpy import pi

from proximity_grid.msg import ProximityGridMsg
from std_srvs.srv import Empty

global straight, turn, stop, running

straight = 0
turn = 0
stop = 0
running = False

coeff_vel_straight = 0.5
coeff_vel_turn = 0.5
mode = 0

def joy_callback(Joy): 
   global turn, straight, stop, running
   turn = Joy.axes[2]
   straight = Joy.axes[3]
   stop = Joy.buttons[2]

   global start_srv, stop_srv

   if (stop != 0 and not running):
        start_srv()
        running = True
   elif (stop != 0 and  running):
        stop_srv()
        running = False
      

def generate_yon():
   global yon, straight, turn, stop
   yon.linear.x = coeff_vel_straight*straight 
   yon.angular.z = turn*coeff_vel_turn
   return

def setup_services():
    global start_srv, stop_srv
    rospy.wait_for_service("/navigation/navigation_start")
    rospy.wait_for_service("/navigation/navigation_stop")
    start_srv = rospy.ServiceProxy("/navigation/navigation_start", Empty)
    stop_srv  = rospy.ServiceProxy("/navigation/navigation_stop",  Empty)

def gen_att(grid):
   global straight, turn, stop

   alpha = turn * pi/2
   l=[]
   d = straight + 3

   # Setup the angle if the point is in the back position
   if alpha < grid.angle_min:
      alpha = grid.angle_min
   elif alpha > grid.angle_max:
      alpha = grid.angle_max 
   
   used = False

   n = int( (grid.angle_max - grid.angle_min) / grid.angle_increment )
   for i in range(n):
      if not used:
            if alpha >= grid.angle_min + i * grid.angle_increment and alpha < grid.angle_min + (i + 1) * grid.angle_increment:
               l.append(d)
               used = True
            else:
               l.append(float('inf'))
      else:
            l.append(float('inf'))  
   
   return l

def setup_grid(): 
   grid = ProximityGridMsg()
   angle_min = rospy.get_param('~angle_min', -2.09) # [rad]   
   angle_max = rospy.get_param('~angle_max',  2.09) # [rad]   
   angle_increment = rospy.get_param('~angle_increment', 0.16) # [rad] 
   range_min = rospy.get_param('~range_min', 0) # [m]   
   range_max = rospy.get_param('~range_max', 6) # [m] 
   frame_id = rospy.get_param('~frame_id', 'fused_scan') 
      
   grid.angle_min = angle_min   
   grid.angle_max = angle_max   
   grid.angle_increment = angle_increment 
   grid.range_min = range_min   
   grid.range_max = range_max
   grid.header.frame_id = frame_id

   return grid, frame_id


if __name__ == '__main__':
   rospy.init_node("joy")   

   # event1 = rospy.Publisher("/events/bus", NeuroEvent, queue_size=1)
   coeff_vel_straight = rospy.get_param("~coeff_vel_straight", coeff_vel_straight)
   coeff_vel_turn     = rospy.get_param("~coeff_vel_turn", coeff_vel_turn)
   mode               = rospy.get_param("~mode", mode)

   global yon, timer
   yon = Twist()
   comandi_utente = NeuroEvent()
   random_num = NeuroEvent()
   wheelchair_move = NeuroEvent()
   var_tmp = NeuroEvent()
   var_tmp.event = 0




   rospy.Subscriber("/joy", Joy, joy_callback, queue_size=1)

   rate = rospy.Rate(50)   

   if (mode == 0):
      cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

      while not rospy.is_shutdown(): 
         generate_yon()
         cmd_pub.publish(yon)
         rate.sleep()

   elif (mode == 1):
      att_pub = rospy.Publisher('/proximity_grid/attractors', ProximityGridMsg, queue_size=1)
      grid, frame_id = setup_grid() 
      setup_services()
   
      while not rospy.is_shutdown(): 
         grid.ranges = gen_att(grid)
         att_pub.publish(grid)
         rate.sleep()


	
	
