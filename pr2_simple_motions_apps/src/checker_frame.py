#!/usr/bin/env python

import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import tf
from geometry_msgs.msg import PoseStamped
import thread
from numpy import *


class CheckerFrameDetector:

    def __init__(self,checkertopic):
        self.output_frame = "checker_frame"
        self.rel_frame = "base_footprint"
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.transform_lock = thread.allocate_lock()
        self.trans = (0,0,0)
        self.rot = (0,0,0,1)
        self.sub = rospy.Subscriber(checkertopic, PoseStamped, self.update_frame)
        self.continually_broadcast()
        
    def update_frame(self,pose):
        now = rospy.Time.now()
        self.listener.waitForTransform(self.rel_frame,"narrow_stereo_optical_frame",now,rospy.Duration(10.0))
        checker_pose = self.listener.transformPose(self.rel_frame,pose)
        trans = (checker_pose.pose.position.x,checker_pose.pose.position.y,checker_pose.pose.position.z)
        rot = (checker_pose.pose.orientation.x,checker_pose.pose.orientation.y,checker_pose.pose.orientation.z,checker_pose.pose.orientation.w)
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)
        #pitch += pi/2
        new_rot = tf.transformations.quaternion_from_euler(0,0,0)
        self.transform_lock.acquire()
        self.trans = trans
        self.rot = new_rot
        self.transform_lock.release()
    
    def continually_broadcast(self):
        while not rospy.is_shutdown():
            self.broadcast()
            rospy.sleep(0.1)
            
    def broadcast(self):
        self.transform_lock.acquire()
        self.broadcaster.sendTransform(self.trans,self.rot,rospy.Time.now(),self.output_frame,self.rel_frame)
        self.transform_lock.release()
        
## Creates a stereo_converter node    
def main(args):
    
    rospy.init_node("checker_frame_detector")
    detector = CheckerFrameDetector(checkertopic="/board_pose")
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
