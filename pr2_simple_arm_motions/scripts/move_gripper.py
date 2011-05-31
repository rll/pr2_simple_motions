#!/usr/bin/env python
import roslib
roslib.load_manifest("pr2_simple_arm_motions")
import rospy
import sys
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped, QuaternionStamped
from pr2_simple_motions_srvs.srv import *
from pr2_common_action_msgs.msg import *
from pr2_controllers_msgs.msg import *
from std_srvs.srv import Empty
from numpy import *
import tf
import time
import thread
import cv
import yaml

GOAL_STATUS_SUCCEEDED = 3

TOOL_TIP_OFFSET = 0.0

INTERPOLATE = False
INTERP_NAME = "CalibData"

"""
Better documentation to come

Uses the ArmMoveIKAction to move the tip of the gripper to a specified pose.

Services:

---
move_one_arm
---
Moves one arm to a specified pose

---
move_both_arms
---
Moves both arms to a specified pose

---
open_grippers
---
Opens the grippers

---
close_grippers
---
Closes the grippers

"""
class SuccessState:
    def __init__(self):
        self.success_flag = False
        
    def succeed(self):
        self.success_flag = True
    
    def fail(self):
        self.success_flag = False
        
    def succeeded(self):
        return self.success_flag

class MoveGripper:
    def __init__(self):
        self.right_arm_lock = thread.allocate_lock()
        self.left_arm_lock = thread.allocate_lock()
        self.cancel_lock = thread.allocate_lock()
        self.arm_locks = [self.right_arm_lock,self.left_arm_lock]
        self.arm_tf_locks = {"l":thread.allocate_lock(),"r":thread.allocate_lock()}
        self.motion_locks = {"l":thread.allocate_lock(),"r":thread.allocate_lock()}
        self.listener = tf.TransformListener()
        self.cancel_srv = rospy.Service('cancel_all_arm_motion',Empty,self.cancel_all)
        self.client_grip_l = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.client_grip_r = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
        self.client_arm_r = actionlib.SimpleActionClient('r_arm_ik', ArmMoveIKAction)
        self.client_arm_l = actionlib.SimpleActionClient('l_arm_ik', ArmMoveIKAction)
        self.client_grip_l.wait_for_server()
        self.client_grip_r.wait_for_server()
        self.client_arm_l.wait_for_server()
        self.client_arm_r.wait_for_server()
        
        #Interpolation
        if INTERPOLATE:
            self.config_dir = "%s/config"%os.path.dirname(os.path.dirname( os.path.realpath( __file__ ) ) )
            self.calib_mat = {}
            self.calib_info = {}
            for arm in ["l","r"]:
                self.calib_mat[arm] = cv.Load(self.config_dir+'/'+INTERP_NAME+'_'+arm+'.mat')
                self.calib_info[arm] = yaml.load(self.config_dir+'/'+INTERP_NAME+'_'+arm+'.info')
        
        
        self.begin_sync_srv = rospy.Service("begin_sync",BeginSync,self.begin_sync)
        self.end_sync_srv = rospy.Service("end_sync",EndSync,self.end_sync)
        self.clear_sync_srv = rospy.Service("clear_sync",ClearSync,self.clear_sync)
        self.move_one_arm_srv = rospy.Service("move_one_arm",MoveOneArm,self.move_one_arm)
        self.move_both_arms_srv = rospy.Service("move_both_arms",MoveBothArms,self.move_both_arms)
        self.open_grippers_srv = rospy.Service("open_grippers",OpenGrippers,self.open_grippers)
        self.close_grippers_srv = rospy.Service("close_grippers",CloseGrippers,self.close_grippers)
#    	for i in range(2):
#    		self.arm_locks.append(thread.allocate_lock())
        self.cancel = False
    
    def begin_sync(self,req):
        print "Beginning sync with arm %s"%req.arm
        self.motion_locks[req.arm].acquire()
        print "Done!"
        return BeginSyncResponse()
    
    def end_sync(self,req):
        print "Ending sync with arm %s"%req.arm
        self.motion_locks[req.arm].release()
        if req.arm=="l":
            self.motion_locks["r"].acquire()
            self.motion_locks["r"].release()
        else:
            self.motion_locks["l"].acquire()
            self.motion_locks["l"].release()
        print "Done!"
        return EndSyncResponse()
        
    def clear_sync(self,req):
        print "Clearing sync with arm %s"%req.arm
        self.motion_locks[req.arm].release()
        print "Done!"
        return ClearSyncResponse()
    
    def cancel_all(self,req):
        self.cancel_if_running(self.client_arm_l)
        return []
    
    def cancel_if_running(self,client):
        self.cancel_lock.acquire()
        self.cancel = True
        self.cancel_lock.release()
        return
        
    def canceled(self):
        self.cancel_lock.acquire()
        output = self.cancel
        self.cancel_lock.release()
        return output
        
            
            
    
    def move_both_arms(self,req):
    	targets = (req.target_left,req.target_right)
    	dur = req.dur
    	success_states = []
    	#self.arm_tf_locks["l"].acquire()
    	#self.arm_tf_locks["r"].acquire()
    	self.begin_sync(BeginSyncRequest("l"))
    	self.begin_sync(BeginSyncRequest("r"))
    	for lock_num,target in enumerate(targets):
            if(not target.empty):
                self.arm_locks[lock_num].acquire()
                success_state = SuccessState()
                success_states.append(success_state)
                thread.start_new_thread(self.move_arm, (target,dur,True,success_state))
        self.synchronize()
        success = success_states[0].succeeded() and success_states[1].succeeded()
        print "Success_state[0] == %s, Success_state[1] == %s. Total success = %s"%(success_states[0].succeeded(),success_states[1].succeeded(),success)
        return MoveBothArmsResponse(success)

    def move_one_arm(self,req):
        self.begin_sync(BeginSyncRequest(req.target.arm))
        success = self.move_arm(req.target,req.dur)
        return MoveOneArmResponse(success)
	
    def move_arm(self,target,dur,sync=False,success_state=None):
        self.cancel = False
        target_point = target.point
        target_frame = target_point.header.frame_id
        grip = target.grip
        pitch = target.pitch
        roll = target.roll
        yaw = target.yaw
        arm = target.arm
        # To ensure compatibility with older code, we default to the tip frame
        if target.link_frame == "":
            link_frame = "%s_tip_frame"%arm
        else:
            link_frame = target.link_frame
        (qx,qy,qz,qw) = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        final_quat = QuaternionStamped()
        final_quat.header.frame_id = target_frame
        final_quat.header.stamp = rospy.Time.now()
        final_quat.quaternion.x = qx
        final_quat.quaternion.y = qy
        final_quat.quaternion.z = qz
        final_quat.quaternion.w = qw
        self.listener.waitForTransform("%s_wrist_roll_link"%arm,target_frame,rospy.Time.now(),rospy.Duration(100.0))
        rel_quat = self.listener.transformQuaternion("%s_wrist_roll_link"%arm,final_quat)
        rel_quat_tuple = (rel_quat.quaternion.x,rel_quat.quaternion.y,rel_quat.quaternion.z,rel_quat.quaternion.w)
        #Then, convert to wrist space
        now = rospy.Time.now()
        target_point.header.stamp = now
        self.listener.waitForTransform(target_frame,"%s_wrist_roll_link"%arm,now,rospy.Duration(100.0))
        time.sleep(0.01)
        wrist_target_point = self.listener.transformPoint("%s_wrist_roll_link"%arm,target_point)      
        self.listener.waitForTransform(link_frame,"%s_wrist_roll_link"%arm,rospy.Time.now(),rospy.Duration(100.0))
        #Get transformation and rotation between the two
        (trans,rot) = self.listener.lookupTransform("%s_wrist_roll_link"%arm, link_frame, rospy.Time(0))
        trans_q = (trans[0]+TOOL_TIP_OFFSET,trans[1],trans[2],0)
        tip_offset = tf.transformations.quaternion_multiply(rel_quat_tuple,tf.transformations.quaternion_multiply(trans_q,tf.transformations.quaternion_inverse(rel_quat_tuple)))
        wrist_target_point.point.x -= tip_offset[0]
        wrist_target_point.point.y -= tip_offset[1]
        wrist_target_point.point.z -= tip_offset[2]
        new_target_point = self.listener.transformPoint(target_frame,wrist_target_point)
        #Finally, move to it
        x = new_target_point.point.x
        y = new_target_point.point.y
        z = new_target_point.point.z
        if arm=="r":
            curlock = self.right_arm_lock
        else:
            curlock = self.left_arm_lock
        if sync:
            #print "SYNCHRONIZING before sending"
            #print "Releasing lock for arm %s"%arm
            #self.arm_tf_locks[arm].release()
            if arm == "l":
                other_arm = "r"
            else:
                other_arm = "l"
            #print "Acquiring then releasing lock for arm %s"%other_arm
            #self.arm_tf_locks[other_arm].acquire()
            #self.arm_tf_locks[other_arm].release()
        print "SENDING GOAL (%f,%f,%f) at orientation (%f,%f,%f) for arm %s"%(x,y,z,roll,pitch,yaw,arm)
        success = self.go_to(x=x,y=y,z=z,arm=arm,grip=grip,pitch=pitch,roll=roll,yaw=yaw,frame=target_frame,dur=dur)
        if sync:
            if success:
                success_state.succeed()
            else:
                success_state.fail()
            curlock.release()
        return success
            
    def go_to(self,x,y,z,arm,pitch,roll,yaw,frame,grip="default",dur=5.0):
        print "Moving %s arm to (%f, %f, %f) at orientation (%f, %f, %f) in frame %s"%(arm,x,y,z,pitch,roll,yaw,frame)
        """ If interpolate, modify x,y,z accordingly """
        if INTERPOLATE:
            now = rospy.Time.now()
            self.listener.waitForTransform(self.calib_info[arm]["base_frame"],frame,now,rospy.Duration(10.0))
            torso_pt = self.listener.transformPoint(self.calib_info[arm]["base_frame"],torso_pt)
            frame = self.calib_info[arm]["base_frame"]
            x = torso_pt.point.x
            y = torso_pt.point.y
            z = torso_pt.point.z
            x_range = self.calib_info[arm]["x_range"]
            y_range = self.calib_info[arm]["y_range"]
            z_range = self.calib_info[arm]["z_range"]
            x_step = x_range[1] - x_range[0]
            y_step = y_range[1] - y_range[0]
            z_step = z_range[1] - z_range[0]
            i = (x - x_range[0]) / float(x_step)
            j = (y - y_range[0]) / float(y_step)
            k = (z - z_range[0]) / float(z_step)
            trans = [0,0,0]
            factors = []
            for ni in (floor(i),ceil(i)):
                for nj in (floor(j),ceil(j)):
                    for nk in (floor(k),ceil(k)):
                        dist = (abs((i-ni)*x_step)+abs((j-nj)*y_step)+abs((k-nk)*z_step))/(abs(x_step)+abs(y_step)+abs(z_step)) #L1 Normalized distance
                        factors.append(1-dist)
            scale = 1.0 / sum(factors)
            for ni in (floor(i),ceil(i)):
                for nj in (floor(j),ceil(j)):
                    for nk in (floor(k),ceil(k)):
                        factor = factors[ni*4+nj*2+nk]*scale
                        for dim in range(3):
                            if ni < 0:
                                ni = 0
                            if ni >= len(x_range):
                                ni = len(x_range)-1
                            if nj < 0:
                                nj = 0
                            if nj >= len(y_range):
                                nj = len(y_range)-1
                            if nk < 0:
                                nk = 0
                            if nk >= len(z_range):
                                nk = len(z_range)-1
                            trans[dim] += factor*self.calib_mat[arm][ni,nj,nk]
            x += trans[0]
            y += trans[1]
            z += trans[2]
        """ End interpolation"""
        
        if arm=="l":
    		client_grip = self.client_grip_l
    	else:
    		client_grip = self.client_grip_r
        if (grip):
            grip_pos = -100.0
        else:
            grip_pos = 1.0
        print "Sending grip goal"
        send_grip = True
        if self.get_current_grip(arm) == grip or grip == "default":
            send_grip = False
            
        if send_grip:
            client_grip.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = grip_pos, max_effort = -1.0)))
        if arm=="r":
            client_arm = self.client_arm_r
            armfull = "right"
        else:
            client_arm = self.client_arm_l
            armfull = "left"
        #curlock.acquire()
        (qx, qy, qz, qw) = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        goal = ArmMoveIKGoal()
        goal.ik_timeout = rospy.Duration(dur*5)
        goal.tool_frame.header.frame_id = "%s_tip_frame" % arm
        goal.pose.header.frame_id = frame
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = z
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        goal.ik_seed.name = [p%arm for p in ['%s_shoulder_pan_joint', '%s_shoulder_lift_joint', '%s_upper_arm_roll_joint', '%s_elbow_flex_joint', '%s_forearm_roll_joint', '%s_wrist_flex_joint', '%s_wrist_roll_joint']]
        if arm=="l":
            goal.ik_seed.position = [0.83007285324949953,-0.35320448964307055,1.7675383848900958,-1.1557956529332751,1.5035863485216983,-1.4448739148710006, -1.5598103297885944]   
        else:
            goal.ik_seed.position = [-0.58382693133119901,-0.35320448964307055,-1.7535368129157851,-0.99857677756936158,-1.2030484613896466,-1.293617529636232,-4.4515510771613656+pi]
        goal.move_duration = rospy.Duration(dur)
        
        finished = client_arm.send_goal_and_wait(goal)
        success = (finished==GOAL_STATUS_SUCCEEDED)
        #print "SENT GOAL with output %s"%output

        #finished = client_arm.wait_for_result()
        print "FINISHED WITH MSG %s"%finished
        if send_grip:
            client_grip.wait_for_result()
        
        return success
        
        
            
            
    def get_current_grip(self,arm):
        resp = False
        try:
            get_joint = rospy.ServiceProxy("return_joint_state",ReturnJointState)
            resp = get_joint("%s_gripper_r_finger_joint"%arm)
        except rospy.ServiceException,e:
            rospy.loginfo("Service Call Failed: %s"%e)
        if resp:
            return resp.position < 0.2
            
    def open_grippers(self,req):
        arms = req.arms
        effort = -1.0
        result = self.command_gripper(close=False,arms=arms,effort=effort)
        return OpenGrippersResponse(result)
        
   
    def close_grippers(self,req):
        arms = req.arms
        effort = -1.0
        result = self.command_gripper(close=True,arms=arms,effort=effort)
        return CloseGrippersResponse(result)
        
        
    def command_gripper(self,close,arms,effort=-1.0):
        if close:
            grip_pos = -100.0
        else:
            grip_pos = 1.0
        clients = []
        if arms == "l" or arms == "b":
            clients.append(self.client_grip_l)
        if arms == "r" or arms == "b":
            clients.append(self.client_grip_r)
        for client in clients:
            client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = grip_pos, max_effort = effort)))
        result = True
        for client in clients:
            result &= client.wait_for_result()
            print "Waited for grip result and got %s"%result
        return result
        
    def synchronize(self):
      for l in self.arm_locks:
        l.acquire()
      time.sleep(0.005)
      for l in self.arm_locks:
        l.release()
             

        
        
def main(args):
    rospy.init_node("gripper_mover")
    gm = MoveGripper()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass
