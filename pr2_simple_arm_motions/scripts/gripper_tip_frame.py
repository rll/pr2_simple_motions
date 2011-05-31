#!/usr/bin/env python  
import roslib
roslib.load_manifest('pr2_simple_arm_motions')

import rospy
import tf

"""
Broadcasts four new frames: l_tip_frame, r_tip_frame, l_grip_frame, and r_grip_frame.
The tip frames should be located exactly at the tip of the gripper, when it is closed.
The grip frames are similar to the gripper_tool_frames, but at more convenient locations for cloth grasping.
"""

if __name__ == '__main__':
    rospy.init_node('gripper_tip_frame_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        for arm in ("l","r"):
            br.sendTransform((0.196, 0.0, 0.0),
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "%s_tip_frame"%arm,
                             "%s_wrist_roll_link"%arm)
            br.sendTransform((0.170, 0.0, 0.0),
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "%s_grip_frame"%arm,
                             "%s_wrist_roll_link"%arm)
            br.sendTransform((-0.04, 0.0, 0.0),
                             (0.0, 0.0, 0.0, 1.0),
                             rospy.Time.now(),
                             "%s_wrist_back_frame"%arm,
                             "%s_wrist_roll_link"%arm)
            br.sendTransform((0.255, 0.0, 0.0),
                            (0.0,0.0,0.0,1.0),
                            rospy.Time.now(),
                            "%s_marker_frame"%arm,
                            "%s_wrist_roll_link"%arm)
        rate.sleep()

