#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State

import sys, select, termios, tty
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

msg = """
Control Your ROSCAR!
---------------------------
Moving around:
        w               i  
    a   s   d       j   k   l
        x               ,

w/x : increase/decrease accell velocity 
a/d : increase/decrease steering velocity 

space key, s : force stop
x : steering init

CTRL-C to quit
"""


''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.stat_sub = rospy.Subscriber('/mavros/state', State, self.stat_callback)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(30)
        self.stat_check = False
        self.pose_check = False
        self.control_init=False
        self.stat = State()
        self.goal = 0

        self.goal_pose = PoseStamped()
        self.yaw = 0

        self.settings = termios.tcgetattr(sys.stdin)

    def stat_callback(self, msg):
        self.stat = msg
        self.stat_check = True
    def pose_callback(self, msg):
        self.pose_check = True
        self.truth=msg.pose.position
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def input(self):
        key = self.getKey()
        if key == 'w' :
            self.goal_pose.pose.position.x +=1
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == 'a' :
            self.goal_pose.pose.position.y +=1
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == 'd' :
            self.goal_pose.pose.position.y -=1
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == 'x' :
            self.goal_pose.pose.position.x -=1
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == 'i' :
            self.goal_pose.pose.position.z +=1
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == ',' :
            self.goal_pose.pose.position.z -=1
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == 'j' :
            self.yaw +=90
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
            self.goal_pose.pose.orientation.x = quaternion[0]
            self.goal_pose.pose.orientation.y = quaternion[1]
            self.goal_pose.pose.orientation.z = quaternion[2]
            self.goal_pose.pose.orientation.w = quaternion[3]
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        elif key == 'l' :
            self.yaw -=90
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
            self.goal_pose.pose.orientation.x = quaternion[0]
            self.goal_pose.pose.orientation.y = quaternion[1]
            self.goal_pose.pose.orientation.z = quaternion[2]
            self.goal_pose.pose.orientation.w = quaternion[3]
            print(self.goal_pose.pose.position.x, self.goal_pose.pose.position.y,self.goal_pose.pose.position.z,self.yaw)
        else:
            if (key == '\x03'):
                print('You pressed Ctrl+C!')
                sys.exit(0)
        # elif key == ' ' or key == 's' :

        # elif key == 'x' :


        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        if self.stat_check and self.pose_check:

            if not self.control_init:
                if not self.stat.armed:
                    self.arming(True)
                elif self.stat.mode!="OFFBOARD":
                    self.offboarding(base_mode=0, custom_mode="OFFBOARD")
                    pose_input=PoseStamped()
                    pose_input.header.stamp=rospy.Time.now()
                    pose_input.pose.orientation.w=1.0
                    self.position_pub.publish(pose_input)
                else:
                    self.control_init=True

            elif self.control_init:
                pose_input=PoseStamped()
                pose_input.header.stamp=rospy.Time.now()

                pose_input.pose.orientation.x = self.goal_pose.pose.orientation.x
                pose_input.pose.orientation.y = self.goal_pose.pose.orientation.y
                pose_input.pose.orientation.z = self.goal_pose.pose.orientation.z
                pose_input.pose.orientation.w = self.goal_pose.pose.orientation.w
                pose_input.pose.position.x=self.goal_pose.pose.position.x
                pose_input.pose.position.y=self.goal_pose.pose.position.y
                pose_input.pose.position.z=self.goal_pose.pose.position.z
                if abs(self.truth.x-self.goal_pose.pose.position.x) < 0.2 and abs(self.truth.y-self.goal_pose.pose.position.y) < 0.2 and abs(self.truth.z-self.goal_pose.pose.position.z) < 0.2:
                    self.goal=1

                self.position_pub.publish(pose_input)

##############################################################################################

mav_ctr = robot()
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    print msg
    
    while 1:
        try:
            mav_ctr.input()
            mav_ctr.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
