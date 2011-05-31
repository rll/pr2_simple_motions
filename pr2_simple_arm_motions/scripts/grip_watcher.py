#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_arm_motions")
import rospy
import sys
from numpy import *
import tf
import time
from geometry_msgs.msg import PoseStamped,PointStamped
from pr2_simple_motions_msgs.msg import *
from pr2_simple_motions_srvs.srv import *

"""
Watches the joint_states of the two grippers, and provides a few services:

---
locate_one_arm
Locates the pose of the tip of one of the grippers
---

---
has_object
Brute force hack: checks the joint state of the gripper to see if it is holding an object or not.
(Assumes fully closed when called; doesn't check to see if the gripper is in motion)
"""

class GripWatcher:

    def __init__(self):
        self.name = rospy.get_name()
        self.listener = tf.TransformListener()
        self.locate_one_arm_server = rospy.Service("locate_one_arm",LocateOneArm,self.locate_one_arm)
        self.has_object_server = rospy.Service("has_object",HasObject,self.has_object_serve)
        
    def locate_one_arm(self,req):
        arm = req.arm
        frame = req.frame
        pose = self.get_pose(arm,frame)
        grip = self.get_grip(arm)
        target = self.pose_to_target(arm,pose,grip)
        return LocateOneArmResponse(target=target)
        
    def get_pose(self,arm,frame):
        origin = self.get_origin("%s_tip_frame"%arm)
        self.listener.waitForTransform(frame,"%s_tip_frame"%arm,rospy.Time.now(),rospy.Duration(4.0))
        return self.listener.transformPose(frame,origin)
        
    def get_origin(self,frame):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time.now()
        return pose
        
    def get_grip(self,arm):
        resp = False
        try:
            get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
            resp = get_joint("%s_gripper_r_finger_joint"%arm)
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        if resp:
            return resp.position < 0.2
            
    def has_object_serve(self,req):
        arm = req.arm
        result = self.has_object(arm)
        return HasObjectResponse(result)
            
    def has_object(self,arm):
        resp = False
        try:
            get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
            resp = get_joint("%s_gripper_l_finger_joint"%arm)
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
            rospy.sleep(0.5)
            return self.has_object(arm)
        if resp:
            if arm == 'l':
                return -0.003 < resp.position < 0.3# or abs(resp.effort) > 0.8
            else:
                return -0.0037 < resp.position < 0.3
            
    def pose_to_target(self,arm,pose,grip):
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(
                                                (pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w))
        point = PointStamped()
        point.header.frame_id = pose.header.frame_id
        point.header.stamp = rospy.Time.now()
        point.point.x = x
        point.point.y = y
        point.point.z = z
        return GripTarget(point=point,arm=arm,grip=grip,pitch=pitch,roll=roll,yaw=yaw)
        

def main(args):
    rospy.init_node("grip_watcher")
    gw = GripWatcher()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
