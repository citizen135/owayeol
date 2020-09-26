#!/usr/bin/env python

import math
import os
import random
import time
import sys
import subprocess
from os.path import expanduser
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot, qWarning
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QMenu, QTreeWidgetItem, QWidget
from PyQt5.QtWidgets import *
from std_msgs.msg import String,Int8
from owayeol.msg import ChangeRobot, YoloResult,RobotState

import rospkg
import rospy
import genpy

from rqt_py_common.extended_combo_box import ExtendedComboBox

number=1
class MyModuleWidget(QWidget):

  
    def __init__(self):
        super(MyModuleWidget, self).__init__()
        self.setObjectName('MyModuleWidget')

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_mypkg'), 'resource', 'My1.ui')
        loadUi(ui_file, self)
        #self.call_button.setIcon(QIcon.fromTheme('call-start'))
        self.robot1.clicked.connect(self.robot1_button_clicked) 
        self.robot2.clicked.connect(self.robot2_button_clicked)
        self.robot3.clicked.connect(self.robot3_button_clicked) 
        self.call_button.clicked.connect(self.call_button_clicked)
        self.cancel_button.clicked.connect(self.cancel_button_clicked)  


        self.call_button1.clicked.connect(self.call_button_clicked1)
        self.cancel_button1.clicked.connect(self.cancel_button_clicked1)  
        self.call_button2.clicked.connect(self.call_button_clicked2)
        self.wait_save.clicked.connect(self.wait_save_button_clicked)  
        #self.call_tb1.clicked.connect(self.call_button_tb1)
        self.call_tb2.clicked.connect(self.call_button_tb2)
        #self.call_tb3.clicked.connect(self.call_button_tb3)
        self.call_robot1_init.clicked.connect(self.call_button_robot1_init)
        self.call_robot2_init.clicked.connect(self.call_button_robot2_init)
        #self.call_robot3_init.clicked.connect(self.call_button_robot3_init)
        self.call_robot1_teleop.clicked.connect(self.call_button_robot1_teleop)
        #self.call_robot2_teleop.clicked.connect(self.call_button_robot2_teleop)
        #self.call_robot3_teleop.clicked.connect(self.call_button_robot3_teleop)
        self.target_stop.clicked.connect(self.target_button_stop)
        self.patrol.clicked.connect(self.patrol_button)
        self.patrol_stop.clicked.connect(self.patrol_stop_button)
        self.init_place.clicked.connect(self.init_place_button)
        #self.lineEdit = QLabel(self)
        self.Robot_Number.setText("robot1")
        self.Main_Command.setText("nav")
        self.Robot_state.setText("nothing")
        self.Battery_State.setText("70%")
        


    def save_settings(self, plugin_settings, instance_settings):
        print('exit')
    def restore_settings(self, plugin_settings, instance_settings):
        print('running~')
    def trigger_configuration(self):
        print('sdfsdf3')

    @Slot()
    def call_button_clicked(self): #map save
        #homedir=expanduser("~")
	  path= '/home/ahn/owayeol'
	  os.chdir(path)
	  print('hi')
	  os.system("ls | wc -l")
	  count=subprocess.check_output('ls | wc -l', shell=True)
	  os.system('mkdir %s/map%d' %(path,int(count)))
	  time.sleep(1)
	  os.system('rosrun map_server map_saver -f %s/map%d/map ' %(path,int(count)) )
	  time.sleep(1)
	  #os.chdir(path)
	  #os.system('mv map%d.yaml %s/map%d' %(int(count),path,int(count)))
	  #time.sleep(1)
	  #os.system('mv map%d.pgm %s/map%d'  %(int(count),path,int(count)))
    @Slot()
    def cancel_button_clicked(self):
	  os.system("rosnode list | grep map_saver* | xargs rosnode kill") 


    @Slot()
    def robot1_button_clicked(self):
	  print("robot1")
	  global number
	  number=1
    @Slot()
    def robot2_button_clicked(self):
	  print("robot2")
	  global number
	  number=2
    @Slot()
    def robot3_button_clicked(self):
	  print("robot3")
	  global number
	  number=3

    @Slot()
    def call_button_robot1_init(self):
	  global number
	  path= '/home/ahn/owayeol'
	  os.chdir(path)
	  os.system("ls | wc -l")
	  num=subprocess.check_output('ls | wc -l', shell=True)
	  n=int(num)-1
	  str(n)
	  st=str(n)
	  os.chdir("%s/map%s" %(path,st))
	  os.system("rosbag record -O wait%d /initialpose &" %number)  
    @Slot()
    def wait_save_button_clicked(self):
	  os.system("rosnode list | grep record* | xargs rosnode kill") 
	  print('save success')

    
    @Slot()
    def call_button_clicked1(self):
	  path= '/home/ahn/owayeol'
	  os.chdir(path)
	  os.system("ls | wc -l")
	  num=subprocess.check_output('ls | wc -l', shell=True)
	  n=int(num)-1
	  st=str(n)
	  os.chdir("%s/map%s" %(path,st))
	  os.system("ls | wc -l")
	  nu=subprocess.check_output('ls | wc -l', shell=True)
	  os.system('mkdir %s/map%s/path%d' %(path,st,int(nu)-4))
	  print('folder create success')

    @Slot()
    def cancel_button_clicked1(self):
	  path= '/home/ahn/owayeol'
	  os.chdir(path)
	  os.system("ls | wc -l")
	  num=subprocess.check_output('ls | wc -l', shell=True)
	  n=int(num)-1
	  str(n)
	  st=str(n)
	  os.chdir("%s/map%s" %(path,st))
	  os.system("ls | wc -l")
	  nu=subprocess.check_output('ls | wc -l', shell=True)
	  os.chdir('%s/map%s/path%d' %(path,st,int(nu)-5))
	  #os.chdir('/home/ahn/path_dir/path%s' %n)
	  os.system("ls | wc -l")
	  numm=subprocess.check_output('ls | wc -l', shell=True)
	  nuu=int(numm)
	  os.system("rosbag record -O waypoint%d /initialpose &" %(nuu+1)) 
    @Slot()
    def call_button_clicked2(self):
	  os.system("rosnode list | grep record* | xargs rosnode kill") 
	  print('33')
    
    @Slot()
    def call_button_robot2_init(self):
	  global number
	  print("%d"%number)
	  rospy.Publisher("/robot1/initialpose",PoseWithCovarianceStamped,queue_size=1 )
	  rospy.Publisher("/robot2/initialpose",PoseWithCovarianceStamped,queue_size=1 )
	  rospy.Publisher("/robot3/initialpose",PoseWithCovarianceStamped,queue_size=1 )
	  masg = rospy.wait_for_message("/initialpose", PoseWithCovarianceStamped)
	  print(masg)
	  #time.sleep(2)
	  goal_publisher = rospy.Publisher("/robot%d/initialpose"%number,PoseWithCovarianceStamped,queue_size=1 )
	  goal_publisher1 = goal_publisher
	  print("check%d"%number)
	  masg.header.stamp =rospy.Time.now()
	  masg.header.frame_id = "map"
	  goal_publisher.publish(masg)
	  print(masg)
	  #goal_publisher.publish(masg)
	  goal_publisher1.publish(masg)
	  goal_publisher1.publish(masg)
	  print("finish%d"%number)

    @Slot()
    def call_button_tb2(self):
	  global number
	  print("%d"%number)
	  rospy.Publisher("/robot1/move_base_simple/goal",PoseStamped,queue_size=1 %number)
	  rospy.Publisher("/robot2/move_base_simple/goal",PoseStamped,queue_size=1 %number)
	  rospy.Publisher("/robot3/move_base_simple/goal",PoseStamped,queue_size=1 %number)
	  masg = rospy.wait_for_message("/move_base_simple/goal", PoseStamped)
	  print(masg)
	  #time.sleep(2)
	  goal_publisher = rospy.Publisher("/robot%d/move_base_simple/goal"%number,PoseStamped,queue_size=1 )
	  goal_publisher1 = goal_publisher
	  print("check%d"%number)
	  masg.header.stamp =rospy.Time.now()
	  masg.header.frame_id = "map"
	  goal_publisher.publish(masg)
	  print(masg)
	  #goal_publisher.publish(masg)
	  goal_publisher1.publish(masg)
	  goal_publisher1.publish(masg)
	  print("robot%d_goal success"%number)

    @Slot()
    def call_button_robot1_teleop(self):
	  global number
	  print("excute robot%d teleop!!"%number)
	  os.system("ROS_NAMESPACE=robot%d rosrun turtlebot3_teleop turtlebot3_teleop_key"%number)
	  print("finish robot%d teleop!!"%number)

    @Slot()
    def target_button_stop(self):
	  print("robot2")
	  global number
	  number=2
	  self.Robot_Number.setText("robot2")
    @Slot()
    def patrol_button(self):
	  print("patrol start")
	  self.Robot_state.setText("patrol")
    @Slot()
    def patrol_stop_button(self):
	  print("patrol_stop")
	  self.Robot_state.setText("patrol_stop")
    @Slot()
    def init_place_button(self):
	  print("init_place")
	  self.Robot_state.setText("init_place")

##clear
stamp=-1
alert_robot=-1
robot_num=[0]*3
wait_num=[0]*3
count=0
##

def ALERT(data):
    global stamp
    global alert_robot
    global catch
    global wait_num
    global count
     
    try: 
        first_wait=wait_num.index(1)
        stamp=data.stamp                            #yolo add
        catch=data.catch

        changrobot_topic="/robot%s/changepath" % first_wait
        #topic /robot%s/maincommand patrol??????
        change_pub = rospy.Publisher(changrobot_topic, Int8, queue_size=1)
        # change.way_robot=data.alert_robot
        # change.wait_robot=first_wait
        # change.path_num=data.path_num
        change_pub.publish(data.path_num)
        # rospy.loginfo(rospy.get_time())
        
    except ValueError:
        rospy.loginfo("no more wait robot")
    
def robotstate(data):
    global robot_num
    global wait_num
    #robot_num[robot_num]=data.
    #wa


if __name__ == '__main__':
    rospy.init_node('PatrolServer')
    rospy.Subscriber("/alert", YoloResult, ALERT)
    rospy.Subscriber("/robotstate", RobotState, robotstate)
    ##clear
    ex=YoloResult()
    ex.stamp=999
    ex.alert_robot=999
    ex.path_num=999
    ex.catch=999
    ALERT(ex)
    ##
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            pass



    



	  
	  

    


    
