#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_head_motions")
import rospy
import sys
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped,QuaternionStamped
from pr2_controllers_msgs.msg import *
from pr2_simple_motions_srvs.srv import *
from numpy import *
import tf
import time
import thread

"""
Better documentation to come

Provides a few simple services for moving the head.

Services:

---
follow_point
---
Tells the head to follow a point

---
follow_frame
---
Tells the head to follow the origin of a given frame
(equivalent to calling FollowPoint with the point as the origin of the frame

---
angle_head
---
Keeps the head at a given angle

---
stop_following
---
Releases the head from following any point.

"""

class HeadMover:
    def __init__(self):
        self.client_head = actionlib.SimpleActionClient('/head_traj_controller/point_head_action',PointHeadAction)
        self.look_at_point_srv = rospy.Service('look_at_point', LookAtPoint, self.look_at_point)
        self.follow_point_srv = rospy.Service('follow_point', FollowPoint, self.follow_point)
        self.follow_frame_srv = rospy.Service('follow_frame', FollowFrame, self.follow_frame)
        self.stop_following_srv = rospy.Service('stop_following', StopFollowing, self.stop_following)
        self.angle_head_srv = rospy.Service('angle_head', AngleHead, self.angle_head)
        self.client_head.wait_for_server()
        self.follow_target = False
        self.listener = tf.TransformListener()
        self.continually_update()
        
    def look_at(self,pt):
        g = PointHeadGoal()
        g.target.header.frame_id = pt.header.frame_id
        g.target.header.stamp = rospy.Time.now()
        g.target.point.x = pt.point.x
        g.target.point.y = pt.point.y
        g.target.point.z = pt.point.z
        g.min_duration = rospy.Duration(1.0)
        g.pointing_frame = "double_stereo_link"
        finished = self.client_head.send_goal_and_wait(g)
        #finished = self.client_head.wait_for_result()
        print "Clinet_Head Returned %d"%finished
        return finished == 3
        
    def look_at_point(self,req):
        print "Called look_at on point (%f,%f,%f) in %s"%(req.target.point.x,req.target.point.y,req.target.point.z,req.target.header.frame_id)
        success = self.look_at(req.target)
        #rospy.sleep(1.0)
        print "Look_at returned %s"%success
        return LookAtPointResponse(success)

    def follow_point(self,req):
        self.follow_target = req.target
        return FollowPointResponse(True)
        
    def follow_frame(self,req):
        target = PointStamped()
        target.header.frame_id = req.frame
        self.follow_target = target
        return FollowFrameResponse(True)
        
    def stop_following(self,req):
        self.follow_target = False
        return StopFollowingResponse(True)
        
    def angle_head(self,req):
        roll = 0
        yaw = req.pan
        pitch = req.tilt
        (qx,qy,qz,qw) = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        final_quat = QuaternionStamped()
        final_quat.header.frame_id = "torso_lift_link"
        final_quat.header.stamp = rospy.Time.now()
        final_quat.quaternion.x = qx
        final_quat.quaternion.y = qy
        final_quat.quaternion.z = qz
        final_quat.quaternion.w = qw
        #Convert the angle to the current stereo frame
        self.listener.waitForTransform("double_stereo_link","torso_lift_link",rospy.Time.now(),rospy.Duration(10.0))
        rel_quat = self.listener.transformQuaternion("double_stereo_link",final_quat)
        rel_quat_tuple = (rel_quat.quaternion.x,rel_quat.quaternion.y,rel_quat.quaternion.z,rel_quat.quaternion.w)
        #Get the point which corresponds to one meter ahead of the stereo frame
        straight_target_q = (1.0,0,0,0)
        #Rotate by the quaternion
        (x,y,z,trash) = tf.transformations.quaternion_multiply(rel_quat_tuple,tf.transformations.quaternion_multiply(straight_target_q,tf.transformations.quaternion_inverse(rel_quat_tuple)))
        new_target = PointStamped()
        new_target.header.stamp = rospy.Time.now()
        new_target.header.frame_id = "double_stereo_link"
        new_target.point.x = x
        new_target.point.y = y
        new_target.point.z = z
        #Now put in torso_lift_link frame
        self.listener.waitForTransform("torso_lift_link","double_stereo_link",rospy.Time.now(),rospy.Duration(10.0))
        target = self.listener.transformPoint("torso_lift_link",new_target)
        #self.follow_target = target
        success = self.look_at(target)
        self.follow_target = False
        return AngleHeadResponse(success)

        
        
    def continually_update(self):
        while(not rospy.is_shutdown()):
            if self.follow_target:
                self.look_at(self.follow_target)
            rospy.sleep(1.0)
        
        
        
def main(args):
    rospy.init_node("head_mover")
    hm = HeadMover()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
