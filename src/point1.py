#!/usr/bin/env python

import rospy
import move_base
import time
from std_msgs.msg import String,Int32,Header
from geometry_msgs.msg import PoseStamped
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import JointState,Range
import math
servox=0
servoy=0
tok=0

Kp=0.0003
Ki=0.0000000000001
Kd=1000000

Kpy=0.00001

start_time = time.time()
errorx_prev = 0.
errory_prev = 0.
time_prev = 0.
goal_publisher = rospy.Publisher('/find', PoseStamped, queue_size=2)
goal = PoseStamped()
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
hello_str = JointState()
hello_str.header = Header()
hello_str.name = ['joint1', 'joint2']
find=0

def ALERT(data):
    global servox
    global servoy
    global tok
    global find
    global time_prev
    global errorx_prev
    global errory_prev
    
    for st in data.bounding_boxes:           
        print(st.Class)
        if (st.Class=="person"):
            tok=0
	    midx=(st.xmin+st.xmax)/2
            midy=(st.ymax+st.ymin)/2
            errorx=320-midx
            errory=240-midy
            dex = errorx-errorx_prev
            dey = errory-errory_prev
            dt = time.time() - time_prev
            controlx=Kp*errorx + Kd*dex/dt + Ki*errorx*dt          
            controly=Kpy*errory + Kd*dey/dt + Ki*errory*dt
            if controlx>0.3:
                controlx=0.3
            elif controlx<-0.3:
                controlx=-0.3
            if controly>0.3:
                controly=0.3
            elif controly<-0.3:
                controly=-0.3
            servox = servox+controlx
            servoy = servoy+controly
            errorx_prev = errorx
            errory_prev = errory
            find=1
            print(Kp*errorx)
	    print(Ki*errorx*dt)
            print(Kd*dex/dt)
            print(servox)
    if (servox<-1.5):
        servox=-1.5
    elif (servox>1.5):
        servox=1.5
    if (servoy<-0.45):
        servoy=-0.45
    elif (servoy>1.5):
        servoy=1.5


def point(data):
    global servox
    global servoy
    global goal

    goal.header.stamp =rospy.Time.now()
    goal.header.frame_id = "base"

    goal.pose.position.x=data.range*math.cos(servox)*math.sin(servoy+1.57)
    goal.pose.position.y=data.range*math.sin(servox)*math.sin(servoy+1.57)
    goal.pose.position.z=0

def talker():
    global servox
    global servoy
    global pub
    global hello_str
    global find

    rate = rospy.Rate(20) # 10hz
    hello_str.header.stamp = rospy.Time.now()
    hello_str.position = [servox, servoy]
    hello_str.velocity = []
    hello_str.effort = []
    pub.publish(hello_str)
    if find==1:
        goal_publisher.publish(goal)
        find=0
    rate.sleep()

def tic(event):
    global tok
    global directio
    global servox
    head=(-1.5,0,1.5)
    tok=tok+1

global tok
    global servox
    global servoy
    tok=tok+1

    if tok>=10:
        tok=0
        servox+=0.2
        if servox>=1.5:
            servox=-1.5






if __name__ == '__main__':
    rospy.init_node('owayeol_cctv_point')
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,ALERT)
    rospy.Subscriber("/range_data",Range,point)
    #rospy.init_node('joint_state_publisher')
    servox=0
    servoy=0

    rospy.Timer(rospy.Duration(0.1), tic)

    while not rospy.is_shutdown():
        try:
            talker()
        except rospy.ROSInterruptException:
            pass