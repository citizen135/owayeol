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
list1=[]
list2=[]
way_last=len(os.walk("%s/owayeol/map23/path1" % (home)).next()[2])
baglist=[0,]

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

if __name__=="__main__":
    rospy.init_node('PatrolServer',anonymous=True)
    waypoint_store()
    patrol(2,3)
    patrol(3,7)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            pass
