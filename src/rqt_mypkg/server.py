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
from python_qt_binding.QtWidgets import QMenu, QTreeWidgetItem, QWidget ,QMessageBox
from PyQt5.QtWidgets import *
from std_msgs.msg import String,Int8
from rqt_mypkg.msg import ChangeRobot, YoloResult,RobotState
from darknet_ros_msgs.msg import BoundingBoxes

import rospkg
import rospy
import genpy

from rqt_py_common.extended_combo_box import ExtendedComboBox

number=1
count=0
#catch
count1=0
count2=0
count3=0
class MyApp(QWidget):

    global catch
    #global count
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
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
 

        buttonReply = QMessageBox.information(
            self, 'Object Detection', "%s Detection \n do you want to record continue?"%catch, 
            QMessageBox.Yes | QMessageBox.Save | QMessageBox.Cancel | QMessageBox.Reset | QMessageBox.No, 
            QMessageBox.No
            )
 
        if buttonReply == QMessageBox.Yes:
            print('Yes clicked.')
        elif buttonReply == QMessageBox.Save:
            print('Save clicked.')
        elif buttonReply == QMessageBox.Cancel: 
            print('Cancel clicked.')
        elif buttonReply == QMessageBox.Close:  
            print('Close clicked.')
        elif buttonReply == QMessageBox.Reset:
            print('Reply clicked.')
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
        #homedir=expanduser("~")
      path= '/home/pjh/owayeol'
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
      self.Robot_Number.setText("robot1")
      number=1
    @Slot()
    def robot2_button_clicked(self):
      print("robot2")
      global number
      self.Robot_Number.setText("robot2")
      number=2
    @Slot()
    def robot3_button_clicked(self):
      print("robot3")
      global number
      self.Robot_Number.setText("robot3")
      number=3

    @Slot()
    def call_button_robot1_init(self):
      global number
      path= '/home/pjh/owayeol'
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
      path= '/home/pjh/owayeol'
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
      path= '/home/pjh/owayeol'
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
      #os.chdir('/home/pjh/path_dir/path%s' %n)
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

def ALERT(data):
    global catch
    global wait_num
    global bounding_boxes
    global count
    global count1
    global rb_num
    flag=False
    flag1=False
    rb_num=3
    #st=data.bounding_boxes[0]
    print("hi")
    #st=data.data
    for st in data.bounding_boxes:           
        print(st.Class)
        if st.Class=="cup":

            flag=True
            count +=1
            print(count)
            if count==10:
                try: 
                    print("robot%d warning!!!!!!!!!!!!" %rb_num)

                    first_wait=wait_num.index(0)+1
                    changrobot_topic="/robot%s/changepath" % first_wait
                    print(first_wait)
                    change_pub = rospy.Publisher(changrobot_topic, Int8, queue_size=1)
                    change_pub.publish(3)
                    change_pub.publish(3)
                    count=0
                except ValueError:
                    rospy.loginfo("no more wait robot") 
                num=1
                #catch=st.Class
                catch=st
                os.chdir('/home/pjh/bag_dir/robot%d' % rb_num)
                while os.path.exists("event%d" %num):
                    num+=1
                os.system("mkdir event%d" %num)
                os.chdir('/home/pjh/bag_dir/robot%d/event%d' %(rb_num,num))
                os.system("rosbag record /cv_camera/image_raw/compressed &")
                app = QApplication(sys.argv)
                MyApp().show()
                

    if flag==False:
            count=0
    #if flag1==False:
    #       count1=0        
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
    rospy.init_node('PatrolServer')
    print("start!!!!!!")
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,ALERT)
    #os.chdir('/home/pjh/bag_dir/robot3')
    #os.system("rosbag record /cv_camera/image_raw/compressed &")  
    #print("robot1 record start!!!!!!!")
    #rospy.Subscriber("/alert", YoloResult, ALERT)
    rospy.Subscriber("/robotstate", RobotState, robotstate)
    ##clear
    #ex=YoloResult()
    #ex.alert_robot=999
    #ex.path_num=999
    #ex.catch=999
    #ALERT(ex)
    ##
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.ROSInterruptException:
            pass
