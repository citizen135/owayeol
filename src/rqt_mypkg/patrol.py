#!/usr/bin/env python
import rosbag
import rospy
import os, sys, select										#input key
import time
import move_base
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Int8
from rqt_mypkg.msg import RobotState
from os.path import expanduser								#find homedir

homedir=expanduser("~")
way_num=1															#current waypoint
pause=False
flag="start"
run=0

map_num=rospy.get_param("mapnum","1")
robot_num=rospy.get_param("robotnum","1")
path_num=rospy.get_param("pathnum","1")

stat=3
way_last=len(os.walk("%s/owayeol/map%s/path%s" % (homedir,map_num,path_num)).next()[2])			#waypoint number

def arriverobot(data):										
	global stat
	stat=data.status.status
	rospy.loginfo(rospy.get_caller_id() + str(stat))

def command(data):
	global flag
	flag=data.data

def changepath(data):
	global path_num
	global pause
	path_num=data.data
	#flag='p'
	pause=True

def patrol_init():
	global homedir
	bag = rosbag.Bag("%s/owayeol/map%s/wait%s.bag" % (homedir,map_num,robot_num))
	goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
	goal = PoseStamped()
	run=0
	for topic, msg, t in bag.read_messages(topics=[]):
		goal.header.stamp =rospy.Time.now()
		goal.header.frame_id = "map"
		goal.pose=msg.pose.pose
		goal_publisher.publish(goal)
		goal_publisher.publish(goal)

def patrol():
	global stat
	global homedir
	global way_num
	global map_num
	global way_last
	#for i in range(1,len(os.walk("%s/bagfiles" % homedir).next()[2])):
	if stat==3:
		bag = rosbag.Bag("%s/owayeol/map%s/path%s/waypoint%s.bag" % (homedir,map_num,path_num,way_num))
		goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=2)
		goal = PoseStamped()
		run=1
		for topic, msg, t in bag.read_messages(topics=[]):
			goal.header.stamp =rospy.Time.now()
			goal.header.frame_id = "map"
			goal.pose=msg.pose.pose
			goal_publisher.publish(goal)
			goal_publisher.publish(goal)
		stat=1
		way_num+=1
		if way_num==way_last:
			way_num=1
			
def pubstate(event):
	global robot_num
	global path_num
	global run
	robotstate_pub = rospy.Publisher('/robotstate', RobotState, queue_size=2)
	robotstate=RobotState()
	robotstate.robot_num=robot_num
	robotstate.path_num=path_num
	robotstate.run=run
	robotstate_pub.publish(robotstate)

	#rate.sleep()
	#way_list.sort()
	#for i in way_list:
	#	os.system("rosbag play ~/bagfiles/%s" % i )
	#os.system("rostopic pub /path_ready std_msgs/Empty -1")

# def waypoint():
	
#       	#pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
#       	rospy.init_node('talker', anonymous = True)
#       	rate = rospy.Rate(10) #10hz
#       	while not rospy.is_shutdown():
#             	hello_str = "hello world %s" % rospy.get_time()
#             	rospy.loginfo(hello_str)
#             	pub.publish(hello_str)
#             	rate.sleep()

if __name__=="__main__":

	rospy.init_node("patrol")
	rospy.Subscriber('/move_base/result', MoveBaseActionResult, arriverobot)
	rospy.Subscriber('/maincommand', String, command)
	rospy.Subscriber('/changepath', Int8, changepath)

	rospy.loginfo("if you initialpose robot, press 'patrol'\n")
	patrol_init()
	rospy.Timer(rospy.Duration(1), pubstate)
	while not rospy.is_shutdown():
		try:
			#print(flag)
			if (flag=='s'):												#stop & start
				pause= not pause
				stat=3
				print("s")
				if (pause==False):
					stop_publisher = rospy.Publisher('/move_base/cancel',GoalID, queue_size=1)
					stop_publisher.publish()
					way_num=way_num-1
					run=0
					if way_num==0:
						way_num=1
				flag="start"
			elif flag=='w':
				print("w")
				run=0
				patrol_init()
			elif flag=='p':
				print("p")
				pause=True
				run=1
			if pause==True:
				patrol()
			
		except rospy.ROSInterruptException:
			patrol_init()
			pass
