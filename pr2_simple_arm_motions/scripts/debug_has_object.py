#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_simple_arm_motions')
import rospy
from sensor_msgs.msg import JointState

def get_gripper_states(joint_states):
    names = joint_states.name
    positions = joint_states.position
    joint_dict = dict(zip(names, positions))
    print 'l gripper width: ', str(joint_dict['l_gripper_joint'])
    print 'r gripper width: ', str(joint_dict['r_gripper_joint'])
    rospy.sleep(1.0)

if  __name__ == '__main__':
    rospy.init_node('debug_has_object')
    sub = rospy.Subscriber('/joint_states', JointState, get_gripper_states)
    rospy.spin()
