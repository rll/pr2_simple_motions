#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_torso_motions")
import rospy
import sys
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped
from pr2_controllers_msgs.msg import *
from pr2_simple_motions_srvs.srv import *
from numpy import *
import tf
import time
import thread

"""
Simple wrapper function for torso_controller/position_joint_action

Services:

---
move_torso
---
Moves the torso to a specified position
"""

class TorsoMover:
    def __init__(self):
        self.client_torso = actionlib.SimpleActionClient('torso_controller/position_joint_action',SingleJointPositionAction)
        self.torso_move_srv = rospy.Service('move_torso', MoveTorso, self.move_torso_srv)
        self.client_torso.wait_for_server()

    def move_torso_srv(self, req):
        return self.move_torso(req.height)
         
    def move_torso(self,height):
        height = req.height
        goal = SingleJointPositionGoal(position=height,min_duration=rospy.Duration(3.0),max_velocity=0.05)
        self.client_torso.send_goal(goal)
        finished_within_time = self.client_torso.wait_for_result(rospy.Duration(50))
        if not finished_within_time:
            self.client_torso.cancel_goal()
        finished = self.client_torso.wait_for_result()
        return MoveTorsoResponse(True)
        
def main(args):
    rospy.init_node("torso_mover")
    tm = TorsoMover()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
