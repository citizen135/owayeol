#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
#from owayeol.msg import Trigger
from geometry_msgs.msg import PoseStamped
def talker():
    rospy.Publisher('/find', PoseStamped, queue_size=1)
    pub = rospy.Publisher('/find', PoseStamped, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        goal = PoseStamped()
        goal.header.stamp =rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x= 4.02384757996
        goal.pose.position.y= 4.50621032715
        goal.pose.position.z= 0.0
        goal.pose.orientation.x= 0.0
        goal.pose.orientation.y= 0.0
        goal.pose.orientation.z= 0.689883018156
        goal.pose.orientation.w= 0.723920866712
        #object_str =  "person"
        print(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
