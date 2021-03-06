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
import genpy
import dropbox,os, pyautogui
import move_base_msgs
from os.path import expanduser
from datetime import datetime
from actionlib_msgs.msg import GoalID
from os.path import expanduser
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Slot, qWarning
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QMenu, QTreeWidgetItem, QWidget ,QMessageBox
from PyQt5.QtWidgets import *
from std_msgs.msg import String,Int8
from owayeol.msg import ChangeRobot, YoloResult,RobotState, Trigger
from relaxed_astar.msg import Meter
from darknet_ros_msgs.msg import BoundingBoxes
from rqt_py_common.extended_combo_box import ExtendedComboBox

#global distance
waypoint_count=14
search_flag = False
stat = 0
number=1
count=0
#catch
count1=0
count2=0
count3=0
find_flag = False
patrol_targetFind_flag = False
tic_flag = False
point = PoseStamped() 
point_count = 0 
home = expanduser("~")  
class MyApp(QWidget):
    #global catch
    global search_flag
    global count
    global patrol_targetFind_flag
    def __init__(self):
        QWidget.__init__(self)
        self.title = 'Warning Warning'
        self.left = 600
        self.top = 100
        self.width = 00
        self.height = 1000
        self.initUI()
 
    def initUI(self):
        global count
        global search_flag
        global tic_flag
        search_flag =False
        global patrol_targetFind_flag
        patrol_targetFind_flag = False
        count=0
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
 

        buttonReply = QMessageBox.information(
            self, 'Object Detection', "person Detection \n do you want to record continue?", 
            QMessageBox.Yes | QMessageBox.Save | QMessageBox.Cancel | QMessageBox.Reset | QMessageBox.No, QMessageBox.No)
        print("@@@@@@@@@@@@@@@@")
        cctv = [7,8,12,15]  
        if buttonReply == QMessageBox.Yes: # no action
            testmessage()
            print('Yes clicked.')
        elif buttonReply == QMessageBox.Save: # 
            print('Save clicked.')
        elif buttonReply == QMessageBox.Cancel: # 
            print('Cancel clicked.')
        elif buttonReply == QMessageBox.Close:  # 
            print('Close clicked.')
        elif buttonReply == QMessageBox.Reset: # patrol start
            tic_flag = True
            print("start patrol")
            #search_flag = True
            robot_first_nav() 
            print("start patrol1")
            start = search_start_position() 
            print("start patrol2")
            go_list = search(start,cctv)
            backup_list=copy.deepcopy(go_list)
            print(backup_list)
            print("start patrol3") 
            search_nearest_position(go_list)
            if patrol_targetFind_flag == False:
                print(patrol_targetFind_flag)
                print(backup_list)
                search_nearest_position(backup_list)
                if patrol_targetFind_flag == False:
                    print("total patrol")
            print("start patrol4") 
            #search_flag = False
            patrol_targetFind_flag = False
        else:
            os.system("rosnode list | grep record* | xargs rosnode kill") 
            print('No clicked.')
            count=0         
            print(count)        

 
class Server(QWidget):

  
    def __init__(self):
        super(Server, self).__init__()
        self.setObjectName('Server')

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('owayeol'), 'resource', 'My1.ui')
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
        self.Main_Command.setText("nothing")
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
      home = expanduser("~")  
      path= home+'/owayeol'
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
    def cancel_button_clicked(self): #map cancel
      os.system("rosnode list | grep map_saver* | xargs rosnode kill") 


    @Slot()
    def robot1_button_clicked(self): #robot 1 select
      print("robot1")
      global number
      self.Robot_Number.setText("robot1")
      number=1
    @Slot()
    def robot2_button_clicked(self): #robot 2 select
      print("robot2")
      global number
      self.Robot_Number.setText("robot2")
      number=2
    @Slot()
    def robot3_button_clicked(self): #robot 3 select
      print("robot3")
      global number
      self.Robot_Number.setText("robot3")
      number=3

    @Slot()
    def call_button_robot1_init(self): #start position
      global number 
      home = expanduser("~")  
      path= home+'/owayeol'
      os.chdir(path)
      os.system("ls | wc -l")
      num=subprocess.check_output('ls | wc -l', shell=True)
      n=int(num)-1
      str(n)
      st=str(n)
      os.chdir("%s/map%s" %(path,st))
      os.system("rosbag record -O wait%d /initialpose &" %number)  
    @Slot()
    def wait_save_button_clicked(self): #start position cancle
      os.system("rosnode list | grep record* | xargs rosnode kill") 
      print('save success')

    
    @Slot()
    def call_button_clicked1(self): 
      home = expanduser("~")  
      path= home+'/owayeol'
      os.chdir(path)
      os.system("ls | wc -l")
      num=subprocess.check_output('ls | wc -l', shell=True)
      n=int(num)-1
      st=str(n)
      os.chdir("%s/map%s" %(path,st))
      os.system("ls | wc -l")
      nu=subprocess.check_output('ls | wc -l', shell=True)
      os.system("test"+nu)
      os.system('mkdir %s/map%s/path%d' %(path,st,int(nu)-1))
      print('folder create success')

    @Slot()
    def cancel_button_clicked1(self):
      home = expanduser("~")  
      path= home+'/owayeol'
      os.chdir(path)
      os.system("ls | wc -l")
      num=subprocess.check_output('ls | wc -l', shell=True)
      n=int(num)-1
      str(n)
      st=str(n)
      os.chdir("%s/map%s" %(path,st))
      os.system("ls | wc -l")
      os.system("hi2")
      nu=subprocess.check_output('ls | wc -l', shell=True)
      os.chdir('%s/map%s/path%d' %(path,st,(int(nu)-3)))
      os.system("ls | wc -l")
      numm=subprocess.check_output('ls | wc -l', shell=True)
      nuu=int(numm)
      os.system("rosbag record -O waypoint%d /initialpose &" %(nuu+1)) 
      #os.system("rosbag record -O waypoint%d /move_base_simple/goal &" %(nuu+1)) 
      
    @Slot()
    def call_button_clicked2(self):
      os.system("rosnode list | grep record* | xargs rosnode kill") 
      print('33')
    
    @Slot()
    def call_button_robot2_init(self):
      global number
      print("%d"%number)
      self.Main_Command.setText("init")
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
      self.Main_Command.setText("goal")
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
      self.Main_Command.setText("teleop")
      global number
      time.sleep(1)
      print("excute robot%d teleop!!"%number)
      os.system("ROS_NAMESPACE=robot%d rosrun turtlebot3_teleop turtlebot3_teleop_key"%number)
      print("finish robot%d teleop!!"%number)

    @Slot()
    def target_button_stop(self):
      print("robot2")
      global number
      number=2
      self.Robot_Number.setText("robot2")
      self.Main_Command.setText("teleop111")
    @Slot()
    def patrol_button(self):
      global number
      print("patrol start")
      self.Robot_state.setText("patrol")
      topicname="/robot%d/maincommand" % number
      print(topicname)
      comm_pub=rospy.Publisher(topicname, String, queue_size=1)
      comm="p"
      comm_pub.publish(comm)
      comm_pub.publish(comm)
    @Slot()
    def patrol_stop_button(self):
      global number
      print("patrol_stop")
      self.Robot_state.setText("patrol_stop")
      topicname="/robot%d/maincommand" % number
      print(topicname)
      comm_pub=rospy.Publisher(topicname, String, queue_size=1)
      comm="s"
      comm_pub.publish(comm)
      comm_pub.publish(comm)
    @Slot()
    def init_place_button(self):
      global number
      print("init_place")
      self.Robot_state.setText("init_place")
      topicname="/robot%d/maincommand" % number
      print(topicname)
      comm_pub=rospy.Publisher(topicname, String, queue_size=1)
      comm="w"
      comm_pub.publish(comm)
      comm_pub.publish(comm)

##clea
wait_num=[0]*3
wait_num[0]=1
path_num=[0]*3
##

global visited 
global arr
global patrol_point_list
global finish_point
arr = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  
      ,[0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0]
      ,[0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0]
      ,[0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0]
      ,[0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0]
      ,[0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0]
      ,[0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1]
      ,[0,1,0,0,0,0,0,0,1,0,0,1,0,0,0,0]
      ,[0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0]
      ,[0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0]
      ,[0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0]
      ,[0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0]
      ,[0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1]
      ,[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0]
      ,[0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1]
      ,[0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0]]  
visited = [False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False] 

def dfs(index):    
    global visited 
    global arr 
    for i in range(1,len(arr)): 
        if arr[index][i]==0 or visited[i]==True: 
            continue
        visited[i]=True 
        dfs(i) 

def search(start,cctv):
    global visited 
    global arr 
    global patrol_point_list 
    global finish_point
    global home 
    flag = False
    if  start in cctv:
        cctv.remove(start)
        flag = True
    patrol_point_list = []  
    finish_point = 0 
    for i in range(1,len(arr)): 
        for j in range(0,len(cctv)):
            if arr[i][cctv[j]]==1:  
                arr[i][cctv[j]]=0
   
    for i in cctv:
        for j in range(0,len(arr)): 
            arr[i][j]=0

    dfs(start)  
        
    max = 0  
    for i in range(1,len(visited)-1):
        count = 0
        if visited[i] == True: 
            patrol_point_list.append(i) 
            for j in range(1,len(visited)-1): 
                print(i)
                print(j)
                if arr[i][j] == 1: 
                    count+=1
            if count > max:
                max = count
                finish_point = i      
    # arr = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  
    #       ,[0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0]
    #       ,[0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0]
    #       ,[0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0]
    #       ,[0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0]
    #       ,[0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0]
    #       ,[0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1]
    #       ,[0,1,0,0,0,0,0,0,1,0,0,1,0,0,0,0]
    #       ,[0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0]
    #       ,[0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0]
    #       ,[0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0]
    #       ,[0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0]
    #       ,[0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1]
    #       ,[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0]
    #       ,[0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1]
    #       ,[0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0]] 
    arr = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  
           ,[0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0]
           ,[0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0]
           ,[0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0]
           ,[0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0]
           ,[0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0]
           ,[0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1]
           ,[0,1,0,0,0,0,0,0,1,0,0,1,0,0,0,0]
           ,[0,0,0,0,0,0,0,1,0,1,1,0,0,0,0,0]
           ,[0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0]
           ,[0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0]
           ,[0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0]
           ,[0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1]
           ,[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0]
           ,[0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1]
           ,[0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0]]   
    visited = [False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False] 

    print("finish point: "+str(finish_point))  
    rospy.Publisher("robot3/move_base_simple/goal",PoseStamped,queue_size=1)
    #bag = rosbag.Bag("/home/ahn/owayeol/map23/path1/waypoint%d.bag" % finish_point)
    bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint6.bag"%home)
    goal_publisher = rospy.Publisher('robot3/move_base_simple/goal', PoseStamped, queue_size=1)
    goal = PoseStamped()
    for topic, msg, t in bag.read_messages(topics=[]):
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose=msg.pose.pose
        goal_publisher.publish(goal)
    if flag == True:
        patrol_point_list.remove(start)
    if finish_point in patrol_point_list:
        patrol_point_list.remove(finish_point)    

    print("patrol points:"+str(patrol_point_list)) 
    return patrol_point_list


def search_nearest_position(waypoints):
        global waypoint_count
        global distance
        global stat
        global home
        stat = 0
        go_point = 0
        rospy.Publisher("/robot2/move_base_simple/goal",PoseStamped,queue_size=1)
        list_length = len(waypoints)
        print(list_length)
        for j in range(1,list_length+1):
            min = 99999
            for i in waypoints:
                bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint%d.bag" %(home,i))
                goal_publisher = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped, queue_size=1)
                goal = PoseStamped()
                for topic, msg, t in bag.read_messages(topics=[]):
                    goal.header.stamp = rospy.Time.now()
                    goal.header.frame_id = "map"
                    goal.pose=msg.pose.pose
                    goal_publisher.publish(goal)
                    time.sleep(0.4)
                    rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
                    stop_publisher = rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
                    stop_publisher.publish()
                    stop_publisher.publish()
                    time.sleep(0.4)    
                    print("waypoint%d: %s"%(i,distance))
                if min > distance :
                    min = distance 
                    goal.header.stamp = rospy.Time.now()
                    go_point = i
            bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint%d.bag" %(home,go_point))        
            goal_publisher = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped, queue_size=1)
            goal = PoseStamped()
            for topic, msg, t in bag.read_messages(topics=[]):
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map"
                goal.pose=msg.pose.pose
                goal_publisher.publish(goal)
                time.sleep(0.2)        
                print("waypoint%d"%go_point)
                print(distance)
            while stat != 3 :
                continue
            stat=0    
            waypoints.remove(go_point)
            print(waypoints)
        print("finish patrol!!!")    
                    
def robot_first_nav():
    global stat
    global goal_publisher
    global point
    print("1 start")
    rospy.Publisher("/robot2/move_base_simple/goal",PoseStamped,queue_size=1)
    #rospy.Publisher("/robot3/move_base_simple/goal",PoseStamped,queue_size=1)
    goal = PoseStamped()
    goal.header.stamp =rospy.Time.now()
    goal.header.frame_id = point.header.frame_id
    goal.pose = point.pose
    goal_publisher.publish(goal)
    bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint16.bag" %(home))
    goal1 = PoseStamped()
    # for topic, msg, t in bag.read_messages(topics=[]):
    #        goal1.header.stamp = rospy.Time.now()
    #        goal1.header.frame_id = "map"
    #        goal1.pose=msg.pose.pose
    #        goal_publisher1.publish(goal1)
    print("1 end")
    while stat != 3 :
        continue
    stat=0    

def ALERT():
    global wait_num
    global count
    global count1
    global rb_num
    global home
    global point_count
    global search_flag
    global patrol_targetFind_flag
    rb_num=3
    print("hi")
    #st=data.data
    #for st in data.bounding_boxes:                
    # if data.object=="person":
    #         flag=True
    #         count +=1
    #         print(count)
    #         if count==20:    
    #             catch=data.object
                # os.chdir('/home/pjh/bag_dir/robot%d' % rb_num)
                # while os.path.exists("event%d" %num):
                #     num+=1
                # os.system("mkdir event%d" %num)
                # os.chdir('/home/pjh/bag_dir/robot%d/event%d' %(rb_num,num))
                # os.system("rosbag record /cv_camera/image_raw/compressed &")
    app = QApplication(sys.argv)
    MyApp().show()            
        
       
    



# def ALERT(data):
#     global catch
#     global wait_num
#     global bounding_boxes
#     global count
#     global count1
#     global rb_num
#     global home
#     flag=False
#     rb_num=3
#     #st=data.bounding_boxes[0]
#     print("hi")
#     #st=data.data
#     #for st in data.bounding_boxes:                
#     if data.object=="person":
#             flag=True
#             count +=1
#             print(count)
#             if count==20:    
#                 catch=data.object
#                 if  search_flag == True:
#                     dbx = dropbox.Dropbox("dxjMLtH7QBsAAAAAAAAAAVcwYRpytwy67lzvhrqItcYOz440Ksh-KNAh8J5TCNo_")
#                     name = datetime.now()
#                     formattedDate = name.strftime("%Y%m%d_%H%M%S")
#                     filename = str(formattedDate)
#                     pyautogui.screenshot('%s/catkin_ws/src/owayeol_patrol/src/%s.png'%(home,filename))
#                     pathname = ("/phoho/%s.png"%filename)
#                     filename= filename+".png"
#                     filename2= filename
#                     with open(filename, "rb") as f:
#                         dbx.files_upload(f.read(), pathname, mode=dropbox.files.WriteMode.overwrite)
#                     shared_URL = dbx.sharing_create_shared_link_with_settings(pathname).url
#                     modified_URL = shared_URL[:-1] + '0'
#                     print(modified_URL)
#                     os.system("""curl -X POST -H "Content-Type: application/json" -d '{"value2":"%s"}' https://maker.ifttt.com/trigger/test1/with/key/cwQ6zkVXNdj2XhaKkTkTBG"""%modified_URL)
#                     os.chdir("%s/catkin_ws/src/owayeol_patrol/src/"%home)
#                     print(filename+" "+filename2)
#                     os.system("mv %s ./photo"%(filename2))
#                     print("target_found!!!!!!!!!!!!!!")
#                     count=0
#                     search_flag == False
#                     #TODO original patrol section return 
#                 else: 
#                 # os.chdir('/home/pjh/bag_dir/robot%d' % rb_num)
#                 # while os.path.exists("event%d" %num):
#                 #     num+=1
#                 # os.system("mkdir event%d" %num)
#                 # os.chdir('/home/pjh/bag_dir/robot%d/event%d' %(rb_num,num))
#                 # os.system("rosbag record /cv_camera/image_raw/compressed &")
#                     app = QApplication(sys.argv)
#                     MyApp().show()
                
#                 # try: 
#                 #     print("robot%d warning!!!!!!!!!!!!" %rb_num)

#                 #     first_wait=wait_num.index(0)+1
#                 #     changrobot_topic="/robot%s/changepath" % first_wait
#                 #     print(first_wait)
#                 #     change_pub = rospy.Publisher(changrobot_topic, Int8, queue_size=1)
#                 #     change_pub.publish(3)
#                 #     change_pub.publish(3)
#                 #     count=0
#                 # except ValueError:
#                 #     rospy.loginfo("no more wait robot") 
#                 # num=1
#                 #catch=st.Class

#     if flag==False:
#             count=0    
"""             
        elif st.Class=="book":
            flag1=True
            count1 +=1
            print(count1)
            if count1==20:
                catch=st.Class
                app = QApplication(sys.argv)
                MyApp().show()
                count1=0
                change_robot(rb_num)
"""   
def robotstate(data):
    global wait_num
    global path_num
    robo=data.robot_num-1
    wait_num[robo]=data.run
    path_num[robo]=data.path_num

def search_start_position():  
        global flag10
        global waypoint_count
        global distance
        global stat
        global home
        stat = 0
        start_position = 1
        start_list = []
        min = 99999
        rospy.Publisher("/robot2/move_base_simple/goal",PoseStamped,queue_size=1)
        for i in range(1,waypoint_count+1):
            print("test")
            bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint%d.bag" %(home,i))
            goal_publisher = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped, queue_size=1)
            goal = PoseStamped()
            for topic, msg, t in bag.read_messages(topics=[]):
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "map"
                goal.pose=msg.pose.pose
                goal_publisher.publish(goal)
                time.sleep(0.1)
                rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
                stop_publisher = rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
                stop_publisher.publish()
                stop_publisher.publish()
                time.sleep(0.1)    
                print("waypoint%d"%i)
                print(distance)
                print(i)
                start_list.append([distance,i])
                print(start_list[0])
        #print("test1")
        start_list.sort()    
        print(start_list[1][1])
        #start_position = start_list[1][1]
        start_position = 1   
        print("start_position: %d"%start_position)    

        bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint%d.bag" %(home,start_position))
        goal_publisher = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped, queue_size=1)
        goal = PoseStamped()
        for topic, msg, t in bag.read_messages(topics=[]):
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose=msg.pose.pose
            goal_publisher.publish(goal)

        while stat != 3 :
            continue  
        stat = 0       
        print(start_position) 
        return start_position        

def arriverobot(data):										
	global stat
	stat=data.status.status
	rospy.loginfo(rospy.get_caller_id() + str(stat))  


        #flag10 = True

def search_start():
        global flag10
        global home
        #print("hi")
        #rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Publisher("/robot2/move_base_simple/goal",PoseStamped,queue_size=1)
        bag = rosbag.Bag("%s/owayeol/map23/path1/waypoint14.bag"%home)
        goal_publisher = rospy.Publisher('/robot2/move_base_simple/goal', PoseStamped, queue_size=1)
        goal = PoseStamped()
        #print(bag)
        for topic, msg, t in bag.read_messages(topics=[]):
            #print(topic)
            #print(msg)
            #print(t)
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose=msg.pose.pose
            goal_publisher.publish(goal)
            #print(goal)
            time.sleep(0.2)
        rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
        stop_publisher = rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
        stop_publisher.publish()
        stop_publisher.publish()
        time.sleep(3)    
        #flag10 = True


def meter(data):
    global distance
    distance = data.meter
    #rospy.Subscriber("/test",Meter,meter)
    #print(distance)
    #rospy.loginfo("I heard %s", data)   

def search_stop():
    global flag10
    print("sadf")
    rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
    stop_publisher = rospy.Publisher('/robot2/move_base/cancel',GoalID, queue_size=1)
    stop_publisher.publish()
    flag10 = False

def tttt(data):
    print(data.object) 

def testmessage():
    obj="person"
    target_url = "https://maker.ifttt.com/trigger/targetFind/with/key/dyNrf2Ttd8gE-vU0rR6vap"
    requests.post(target_url,data={"value1" : obj})

def Receive_Point(data):
    global point
    global point_count
    global find_flag
    find_flag = True
    point_count = 0
    data.pose.position.z = 0
    point = data

def tic(event):
    global point_count
    global find_flag
    global tic_flag
    point_count+=1
    #print("test1")
    if find_flag == True and point_count == 50:
        find_flag = False
        point_count = 0
        ALERT()
        tic_flag = False
    if point_count == 100:
        point_count = 0

def tic2(event):
    global tic_flag
    global find_flag
    global point_count
    if tic_flag == True and find_flag == True:
        point_count+=1
        print(point_count)
        print(tic_flag)
        if point_count == 50:
            send_message()
            point_count = 0
        
def send_message():
    global patrol_targetFind_flag
    global count 
    global tic_flag
    global find_flag
    patrol_targetFind_flag = True
    find_flag = False
    dbx = dropbox.Dropbox("dxjMLtH7QBsAAAAAAAAAAVcwYRpytwy67lzvhrqItcYOz440Ksh-KNAh8J5TCNo_")
    name = datetime.now()
    formattedDate = name.strftime("%Y%m%d_%H%M%S")
    filename = str(formattedDate)
    pyautogui.screenshot('%s/catkin_ws/src/owayeol/src/%s.png'%(home,filename))
    pathname = ("/phoho/%s.png"%filename)
    filename= filename+".png"
    filename2= filename
    with open(filename, "rb") as f:
        dbx.files_upload(f.read(), pathname, mode=dropbox.files.WriteMode.overwrite)
    shared_URL = dbx.sharing_create_shared_link_with_settings(pathname).url
    modified_URL = shared_URL[:-1] + '0'
    print(modified_URL)
    findpoints="371.234,234.345"
    os.system("""curl -X POST -H "Content-Type: application/json" -d '{"value1":"%s","value2":"%s"}' https://maker.ifttt.com/trigger/test1/with/key/cwQ6zkVXNdj2XhaKkTkTBG"""%(findpoints,modified_URL))
    os.chdir("%s/catkin_ws/src/owayeol/src/"%home)
    print(filename+" "+filename2)
    os.system("mv %s ./photo"%(filename2))
    print("target_found!!!!!!!!!!!!!!")
    count=0
    print(tic_flag)
    #TODO original patrol section return 

"""
def change_robot(data):
    global catch
    global wait_num
    global bounding_boxes
    global count
    #global count1
    global rb_num
    try: 
        print("robot%d warning!!!!!!!!!!!!" %rb_num)

        first_wait=wait_num.index(1)
        hangrobot_topic="/robot%s/changepath" % first_wait
        print(first_wait)
        change_pub = rospy.Publisher(changrobot_topic, Int8, queue_size=1)
        change_pub.publish(path_num[rb_num])
            if count==20:    
    except ValueError:
        rospy.loginfo("no more wait robot") 

def Warningg(data):
    global bounding_boxes
    global count
    st=data.bounding_boxes[0]
    #print(data.bounding_boxes[0].Class)
    #for st in data.bounding_boxes:
    print(st.Class)
    if st.Class=="person":
            print("1111")
            count +=1
            if count==20:
                rospy.Subscriber("/alert", YoloResult, ALERT)
                count=0
                app = QApplication(sys.argv)
                MyApp().show()
                #sys.exit(app.exec_())
    else:
        count=0
    
"""
if __name__ == '__main__':
    global rb_num
    global flag11
    rospy.init_node('PatrolServer',anonymous=True)
    print("start!!!!!!")
    rospy.Subscriber("/test",Meter,meter,queue_size=1)
    #patrol()
    # start = 5 
    # cctv=[5,7,8,9]  
    # search(start,cctv)
    #rospy.Subscriber("/trigger",Trigger,ALERT)     
    rospy.Subscriber("/find",PoseStamped,Receive_Point)
    rospy.Timer(rospy.Duration(0.1),tic) 
    rospy.Timer(rospy.Duration(0.1),tic2)       
    rospy.Subscriber('/robot2/move_base/result', MoveBaseActionResult, arriverobot)
    rospy.Publisher("robot3/move_base_simple/goal",PoseStamped,queue_size=1)     
    goal_publisher = rospy.Publisher('robot2/move_base_simple/goal', PoseStamped, queue_size=1)      
    goal_publisher1 = rospy.Publisher('robot3/move_base_simple/goal', PoseStamped, queue_size=1)      
    rospy.Publisher('robot2/move_base/cancel',GoalID, queue_size=1)        
    #rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,ALERT)
    #os.chdir('/home/pjh/bag_dir/robot3')
    #os.system("rosbag record /cv_camera/image_raw/compressed &")  
    #print("robot1 record start!!!!!!!")
    #rospy.Subscriber("/alert", YoloResult, ALERT)
    rospy.Subscriber("/robotstate", RobotState, robotstate)
    #search_start()
    #search_stop()
    ##clear
    #ex=YoloResult()
    #ex.alert_robot=999
    #ex.path_num=999
    #ex.catch=999
    #ALERT(ex)
    ##
    rospy.spin()
    # while not rospy.is_shutdown():
    #     try:
    #         pass
    #     except rospy.ROSInterruptException:
    #         pass
