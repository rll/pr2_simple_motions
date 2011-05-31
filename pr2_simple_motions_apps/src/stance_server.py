#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_motions_apps")
import rospy
import sys
import actionlib
from pr2_simple_motions_srvs.srv import *
from pr2_simple_motions_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from numpy import *
import tf
import time
import thread
import yaml
import os.path
from pr2_simple_arm_motions import GripUtils

"""
Better documentation to come. For ease of use, see StanceUtils.py

The StanceServer is a means of recording previously seen robot positions, to call later (rather than putting actual
numbers in the code). It provides a few services:


/stances/add_stance
---
Adds a stance to the server.
---

/stances/add_current_stance
---
Takes a snapshot of the robot w.r.t. a given frame, and adds it to the server
---

/stances/save_stances
---
Saves stances to a yaml file, to be loaded later. By default, these are saved in the pr2_simple_motions_apps/config folder.
In the future, they should probably be saved to a local "~/.stances" directory instead, so write permissions don't get messed up.
The naming system is: [name]_stances.yaml. e.g. save_stances("folding") will generate the file /config/folding_stances.yaml
---

/stances/load_stances
---
Loads all stances from a yaml file. You provide the name (e.g. "folding") and it will load from /config/[name]_stances.yaml
This will overwrite any stances which share the same name, but will not remove anything else. 
---

Finally, for every stance which has been made, there is a new service generated. If I have a stance "arms_up", there will be a service
/stances/arms_up
Which takes two arguments: a duration (e.g. 5.0) and a list of parameters. At the moment you can't define parametrized stances, so an empty list will suffice.
e.g.:
rosservice call /stances/arms_up 5.0 []
Will cause the robot to move to the arms_up stance at a duration of 5 seconds.

NOTE: requires the tip_frames to be publishing. See pr2_simple_arm_motions/src/gripper_tip_frame.py
"""

class StanceServer:
    def __init__(self):
        self.stances = {}
        self.listener = tf.TransformListener()
        self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
        self.add_stance_service = rospy.Service('stances/add_stance', AddStance, self.add_stance_srv)
        self.add_current_stance_service = rospy.Service('stances/add_current_stance', AddCurrentStance, self.add_current_stance_srv)
        self.add_default_stances()
        self.save_stances_service = rospy.Service('stances/save_stances', SaveStances, self.save_stances_srv)
        self.load_stances_service = rospy.Service('stances/load_stances', LoadStances, self.load_stances_srv)
        initial_stances = rospy.get_param("~initial_stances","default")
        if initial_stances != "default":
            self.load_stances_srv(LoadStancesRequest(initial_stances))
        rospy.loginfo("Stance Server is ready!")
        
    def add_default_stances(self):
        rospy.Service("stances/pause", ExecuteStance, self.pause)
        rospy.Service("stances/open_left", ExecuteStance, lambda req: GripUtils.open_gripper("l"))
        rospy.Service("stances/open_right", ExecuteStance, lambda req: GripUtils.open_gripper("r"))
        rospy.Service("stances/open_both", ExecuteStance, lambda req: GripUtils.open_gripper("b"))
        rospy.Service("stances/close_left", ExecuteStance, lambda req: GripUtils.close_gripper("l"))
        rospy.Service("stances/close_right", ExecuteStance, lambda req: GripUtils.close_gripper("r"))
        rospy.Service("stances/close_both", ExecuteStance, lambda req: GripUtils.close_gripper("b"))
        self.default_stances = ()
    
    def pause(self,req):
        rospy.sleep(req.dur)
        return ExecuteStanceResponse(True)
        
    def add_stance_srv(self,req):
        stance_name = req.name
        stance = req.stance
        return self.add_stance(stance_name,stance)
        
        
    def add_current_stance_srv(self,req):
        stance_name = req.name
        arms = req.arms
        frame = req.frame
        x_left=y_left=z_left=roll_left=pich_left=yaw_left=x_right=y_right=z_right=roll_right=pitch_right=yaw_right=0
        pitch_left = 0
        grip_left=grip_right=False
        if arms == "l" or arms == "b":
            (x_left,y_left,z_left,roll_left,pitch_left,yaw_left,grip_left) = self.current_pose("l",frame)
        if arms == "r" or arms == "b":
            (x_right,y_right,z_right,roll_right,pitch_right,yaw_right,grip_right) = self.current_pose("r",frame)
        if req.include_torso:
            torso_height = self.get_current_torso()
            set_torso_height = True
        else:
            torso_height = 0.0
            set_torso_height = False
        if req.include_head:
            (head_pan,head_tilt) = self.get_current_head()
            set_head_angle = True
        else:
            head_pan = 0.0
            head_tilt = 0.0
            set_head_angle = False
        stance = Stance (x_left=x_left,y_left=y_left,z_left=z_left,roll_left=roll_left,pitch_left=pitch_left,yaw_left=yaw_left,grip_left=grip_left
                        ,x_right=x_right,y_right=y_right,z_right=z_right,roll_right=roll_right,pitch_right=pitch_right,yaw_right=yaw_right,grip_right=grip_right
                        ,frame_left=frame, frame_right=frame, arms=arms
                        ,set_torso_height=set_torso_height,torso_height=torso_height
                        ,set_head_angle=set_head_angle,head_pan=head_pan,head_tilt=head_tilt)
        return self.add_stance(stance_name,stance)
        
    def add_stance(self,stance_name,stance):
        print "Adding stance '%s':\n%s"%(stance_name,stance)
        #my_stance = self.copy_stance(stance)
        my_stance = stance
        #If the stance has not been added before, we make a server
        if stance_name not in self.stances.keys():
              rospy.Service("stances/%s"%stance_name, ExecuteStance, lambda req: self.execute_stance(self.stances[stance_name],req))
        self.stances[stance_name]=my_stance
        return []
        
    def current_pose(self,arm,frame):
        pose = PoseStamped()
        grip_frame = "%s_tip_frame"%arm
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = grip_frame
        self.listener.waitForTransform(frame,grip_frame,rospy.Time.now(),rospy.Duration(1.0))
        new_pose = self.listener.transformPose(frame,pose)
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion((new_pose.pose.orientation.x,new_pose.pose.orientation.y,new_pose.pose.orientation.z,new_pose.pose.orientation.w))
        x = float(new_pose.pose.position.x)
        y = float(new_pose.pose.position.y)
        z = float(new_pose.pose.position.z)
        grip = self.get_current_grip(arm)
        return (x,y,z,roll,pitch,yaw,grip)
        
    def get_current_grip(self,arm):
        resp = False
        try:
            get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
            resp = get_joint("%s_gripper_r_finger_joint"%arm)
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        if resp:
            return resp.position < 0.2
            
    def get_current_torso(self):
        resp = False
        try:
            get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
            resp = get_joint("torso_lift_joint")
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        if resp:
            return resp.position
            
    def get_current_head(self):
        tilt_resp = pan_resp = False
        try:
            get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
            tilt_resp = get_joint("head_tilt_joint")
            pan_resp = get_joint("head_pan_joint")
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        if tilt_resp and pan_resp:
            return (pan_resp.position,tilt_resp.position)
    
    def execute_stance(self,stance,req):
        dur = req.dur
        if stance.arms == "l" or stance.arms == "b":
            target_left = GripTarget()
            target_left.point.header.frame_id = stance.frame_left
            target_left.point.header.stamp = rospy.Time.now()
            target_left.point.point.x = stance.x_left
            target_left.point.point.y = stance.y_left
            target_left.point.point.z = stance.z_left
            target_left.roll = stance.roll_left
            target_left.pitch = stance.pitch_left
            target_left.yaw = stance.yaw_left
            target_left.grip = stance.grip_left
            target_left.arm = "l"
        if stance.arms == "r" or stance.arms == "b":
            target_right = GripTarget()
            target_right.point.header.frame_id = stance.frame_right
            target_right.point.header.stamp = rospy.Time.now()
            target_right.point.point.x = stance.x_right
            target_right.point.point.y = stance.y_right
            target_right.point.point.z = stance.z_right
            target_right.roll = stance.roll_right
            target_right.pitch = stance.pitch_right
            target_right.yaw = stance.yaw_right
            target_right.grip = stance.grip_right
            target_right.arm = "r"
        success = True
        print "Preparing to send goal..."
        if stance.set_torso_height:
            try:
                srv = rospy.ServiceProxy("move_torso",MoveTorso)
                resp = srv(MoveTorsoRequest(height=stance.torso_height))
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
                success = False
        if stance.set_head_angle:
            try:
                srv = rospy.ServiceProxy("angle_head",AngleHead)
                resp = srv(AngleHeadRequest(pan=stance.head_pan,tilt=stance.head_tilt))
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
                success = False     
        if stance.arms == "b":
            print "Getting both goals..."
            try:
                srv = rospy.ServiceProxy("move_both_arms",MoveBothArms)
                resp = srv(MoveBothArmsRequest(target_left = target_left,target_right=target_right,dur=dur))
                success &= resp.success
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
                success = False
        elif stance.arms == "r":
            try:
                srv = rospy.ServiceProxy("move_one_arm",MoveOneArm)
                resp = srv(MoveOneArmRequest(target=target_right,dur=dur))
                success &= resp.success
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
                success = False
        elif stance.arms == "l":
            try:
                srv = rospy.ServiceProxy("move_one_arm",MoveOneArm)
                resp = srv(MoveOneArmRequest(target=target_left,dur=dur))
                success &= resp.success
            except rospy.ServiceException,e:
                rospy.loginfo("Service Call Failed: %s"%e)
                success = False
        print "DONE!"
        return ExecuteStanceResponse(success)
        
    def copy_stance(self,stance):
        new_stance = Stance()
        new_stance.arms = stance.arms
        new_stance.x_left = stance.x_left
        new_stance.y_left = stance.y_left
        new_stance.z_left = stance.z_left
        new_stance.roll_left = stance.roll_left
        new_stance.pitch_left = stance.pitch_left
        new_stance.yaw_left = stance.yaw_left
        new_stance.grip_left = stance.grip_left
        new_stance.frame_left = stance.frame_left
        new_stance.x_right = stance.x_right
        new_stance.y_right = stance.y_right
        new_stance.z_right = stance.z_right
        new_stance.roll_right = stance.roll_right
        new_stance.pitch_right = stance.pitch_right
        new_stance.yaw_right = stance.yaw_right
        new_stance.grip_right = stance.grip_right
        new_stance.frame_right = stance.frame_right
        return new_stance
        
    def get_new_stances(self):
        stances = {}
        for k,v in self.stances.items():
            if not k in self.default_stances:
                stances[k] = v
        return stances

    def save_stances_srv(self,req):
        filename = "%s_stances.yaml"%req.label
        filepath = "%s/%s"%(self.config_dir,filename)
        savefile = open(filepath,'w')
        yaml.dump(self.dict_to_yaml(self.get_new_stances()),savefile)
        return SaveStancesResponse()
        
    def load_stances_srv(self,req):
        filename = "%s_stances.yaml"%req.label
        filepath = "%s/%s"%(self.config_dir,filename)
        savefile = open(filepath)
        new_stances = self.yaml_to_dict(yaml.load(savefile))
        for k,v in new_stances.items():
            self.add_stance(k,v)
        return LoadStancesResponse()

    def dict_to_yaml(self,stance_dict):
        newdict = {}
        for name,stance in stance_dict.items():
            newdict[name] = {
                            'arms':stance.arms,
                            'x_left':stance.x_left,'y_left':stance.y_left,'z_left':stance.z_left,
                            'roll_left':stance.roll_left,'pitch_left':stance.pitch_left,'yaw_left':stance.yaw_left,
                            'grip_left':stance.grip_left,'frame_left':stance.frame_left,
                            'x_right':stance.x_right,'y_right':stance.y_right,'z_right':stance.z_right,
                            'roll_right':stance.roll_right,'pitch_right':stance.pitch_right,'yaw_right':stance.yaw_right,
                            'grip_right':stance.grip_right,'frame_right':stance.frame_right,
                            'set_torso_height':stance.set_torso_height,'torso_height':stance.torso_height,
                            'set_head_angle':stance.set_head_angle,'head_pan':stance.head_pan,'head_tilt':stance.head_tilt
                            }
        return newdict
    
    def yaml_to_dict(self,stance_yaml):
        newdict = {}
        for key,value in stance_yaml.items():
            new_stance = Stance()
            for k,v in value.items():
                setattr(new_stance,k,v)
            newdict[key] = new_stance
        return newdict
        
def main(args):
    rospy.init_node("stance_server")
    ss = StanceServer()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
