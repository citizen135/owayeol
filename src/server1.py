#!/usr/bin/env python

import requests
import math
import random
import time
import sys
import copy
import subprocess
import move_base
import rospkg
import rosbag
import rospy
import os, sys, select										#input key
import time
import move_base
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Int8
from owayeol.msg import RobotState
from os.path import expanduser

home = expanduser("~")  
total_list1=[3,2,1,7,8,9]
total_list2=[4,5,6,12,11,10]
way_last=len(os.walk("%s/owayeol/map23/path1" % (home)).next()[2])
baglist=[0,]
robot2_waynum=0
robot3_waynum=0

def waypoint_store():
    global baglist
    global way_last

    for i in range(1,way_last+1):
        bag=rosbag.Bag("%s/owayeol/map23/path1/waypoint%d.bag" %(home,i))
        for topic, msg, t in bag.read_messages(topics=[]):
            baglist.append(msg.pose.pose)
        print(str(i))
        print(baglist[i])

def patrol(robotnum,waypointnum):
    global baglist

    goal_publisher = rospy.Publisher('/robot%s/move_base_simple/goal'%(robotnum), PoseStamped, queue_size=1)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose=baglist[waypointnum]
    time.sleep(0.1)
    goal_publisher.publish(goal)

def arriverobot2(data):										
	global stat2
	stat2=data.status.status
	rospy.loginfo(rospy.get_caller_id() + str(stat2))  

def arriverobot3(data):										
	global stat3
	stat3=data.status.status
	rospy.loginfo(rospy.get_caller_id() + str(stat3))  


if __name__=="__main__":
    rospy.init_node('PatrolServer',anonymous=True)
    rospy.Subscriber('/robot2/move_base/result', MoveBaseActionResult, arriverobot2)
    rospy.Subscriber('/robot3/move_base/result', MoveBaseActionResult, arriverobot3)
    waypoint_store()
    patrol(2,3)
    patrol(3,7)
    stat2=3
    stat3=3

    while not rospy.is_shutdown():
        try:
            if stat2==3:
                patrol(2,total_list1[robot2_waynum])
                stat2=0
                robot2_waynum+=1
                if len(total_list1)==robot2_waynum:
                    total_list1.reverse()
                    robot2_waynum=1

            if stat3==3:
                patrol(3,total_list2[robot3_waynum])
                stat3=0
                robot3_waynum+=1
                if len(total_list2)==robot3_waynum:
                    total_list2.reverse()
                    robot3_waynum=1
        except rospy.ROSInterruptException:
            pass
