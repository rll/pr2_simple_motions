#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_base_motions")
import rospy
import sys
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from pr2_simple_motions_srvs.srv import *
from std_srvs.srv import Empty
from numpy import *
import tf
import time
import thread
import math

"""
Better documentation to come

Provides some simple services for rotating the base. For x-y movement check base_move.py.
WARNING: Does not check for collisions! If arms are not directly above base, the rotation could smack something.

Services:

---
rotate_base
---
Rotate the base by a relative yaw (in radians) counterclockwise. Will do minimal work. For example a 2pi rotation will do nothing.

"""

def sign(num):
    if num > 0:
        return 1
    elif num < 0:
        return -1
    else:
        return 0

class BaseRotater:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.base_rotate_srv = rospy.Service('rotate_base', RotateBase, self.rotate_base)
        self.base_pub = rospy.Publisher("base_controller/command",Twist)
         
    def rotate_base(self,req):
        rospy.loginfo("Received new base rotation request")
        vel = 0.15
        goal = self.goal(req.yaw)
        print "Goal: ", goal
        rotating = True

        while(rotating):
            ang_displ = self.goal_displ(goal)
            
            while ang_displ > math.pi:
                ang_displ -= 2*math.pi
            while ang_displ <= -math.pi:
                ang_displ += 2*math.pi

            print "Still have %f to go"%ang_displ
            
            if rotating:
                if abs(ang_displ) <= 0.02: #about 1 degrees off is okay
                    rotating = False
                else:
                    ang_amt = vel * sign(ang_displ)
            if rotating:
                command = Twist()
                command.angular.z = ang_amt
                self.base_pub.publish(command)
            rospy.sleep(0.01)   
        self.base_pub.publish(Twist()) #Stops  
        #I think this needs to change.
        return RotateBaseResponse(True)
    
    #unsure what this is used for
    def current_position(self):
        origin = PointStamped()
        origin.header.frame_id = "base_footprint"
        now = rospy.Time.now()
        self.listener.waitForTransform("odom_combined","base_footprint",rospy.Time.now(),rospy.Duration(2.0))
        origin.header.stamp = now
        current_point = self.listener.transformPoint("odom_combined",origin)
        return (current_point.point.x,current_point.point.y)
   
    def rotate_goal(self,yaw):
        goal = PoseStamped()
        goal.header.frame_id = "base_footprint"
        now = rospy.Time.now()


        goal.pose.orientation.z = sin(yaw/2)
        #using the commented version makes negative angles not work (does the positive angle instead)
        #goal.pose.orientation.z = sign(yaw)*sin(yaw/2)
        goal.pose.orientation.w = cos(yaw/2)

        self.listener.waitForTransform("odom_combined","base_footprint",now,rospy.Duration(2.0))
        goal.header.stamp = now
        goal_point = self.listener.transformPose("odom_combined",goal)
        return goal_point
        
    def rotate_goal_displ(self,orig_goal):
        now = rospy.Time.now()
        self.listener.waitForTransform("base_footprint","odom_combined",now,rospy.Duration(2.0))
        orig_goal.header.stamp = now
        displ = self.listener.transformPose("base_footprint",orig_goal)
        
        angle = sign(displ.pose.orientation.z)*2*math.acos(displ.pose.orientation.w)
        return angle

def main(args):
    rospy.init_node("base_rotater")
    bm = BaseRotater()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
