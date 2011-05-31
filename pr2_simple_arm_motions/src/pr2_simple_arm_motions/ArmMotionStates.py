import roslib
roslib.load_manifest("pr2_simple_arm_motions")
from pr2_simple_arm_motions import GripUtils
from smach import State, StateMachine
from smach_ros import SimpleActionState
from numpy import pi
import rospy
SUCCESS = 'success'
FAILURE = 'failure'

DEFAULT_OUTCOMES = [SUCCESS, FAILURE]

class StateMachineAddition(object):
    def __init__(self, title, state, transitions=None, remapping=None):
        self.title = title
        self.state = state
        self.transitions = transitions
        self.remapping = remapping 

class NestedStateMachine(object):
    def __init__(self, title, transitions=None, state_machine_additions=[], outcomes=DEFAULT_OUTCOMES, state_machines=[], things_to_add = [], input_keys = [], output_keys = [], remapping = None):
        self.title = title
        self.transitions = transitions
        self.state_machine_additions = list(state_machine_additions)
        self.outcomes = outcomes
        self.state_machines = list(state_machines)
        self.things_to_add = list(things_to_add)
        self.input_keys = input_keys
        self.output_keys = output_keys
        self.remapping = remapping
    def add_state(self,title, state, transitions=None, remapping=None):
        self.state_machine_additions.append(StateMachineAddition(title, state, transitions, remapping))
        self.things_to_add.append('s')
    def add_state_machine(self, state_machine):
        self.state_machines.append(state_machine)
        self.things_to_add.append('m')
    def add_states(self):
        sm_nested = StateMachine(outcomes=self.outcomes, input_keys=self.input_keys, output_keys=self.output_keys)
        machine_count = 0
        state_count = 0
        StateMachine.add(self.title,sm_nested,transitions=self.transitions, remapping=self.remapping)
        with sm_nested:
            for thing in self.things_to_add:
                if thing == 's':
                    addition = self.state_machine_additions[state_count]
                    StateMachine.add(addition.title, addition.state, addition.transitions, remapping=addition.remapping)
                    state_count += 1
                else:
                    machine = self.state_machines[machine_count]
                    machine.add_states()
                    machine_count += 1

class ArmsUpThenShakeMachine(NestedStateMachine):
    def __init__(self, title, transitions=None):
        NestedStateMachine.__init__(self, title, transitions=transitions)
        self.add_state('ArmsUp', ArmsUp(), {SUCCESS:'Shake', FAILURE:FAILURE})
        self.add_state('Shake', ShakeOneArm('l',2), {SUCCESS:SUCCESS, FAILURE:FAILURE})

class SuccessFailureState(State):
    def __init__(self,input_keys=[],output_keys=[]):
        State.__init__(self, DEFAULT_OUTCOMES,input_keys=input_keys,output_keys=output_keys)


class Smooth(SuccessFailureState):
    def __init__(self, arm="b", input_keys=['location', 'distance']):
        SuccessFailureState.__init__(self, input_keys=input_keys)
        self.arm = arm
    def execute(self, userdata):
        self.location = userdata.location
        self.distance = userdata.distance
        initial_separation = 0.11
        if self.arm=="b":
            #Put arms together, with a gap of initial_separation between grippers
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,y_offset_l=initial_separation/2.0,z_offset_l=0.05
                    ,link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,y_offset_r=-1*initial_separation/2.0,z_offset_r=0.05
                    ,link_frame_r="r_wrist_back_frame",dur=4.0):
                return FAILURE
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,y_offset_l=initial_separation/2.0,z_offset_l=-0.03, 
                    link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,y_offset_r=-1*initial_separation/2.0,z_offset_r=-0.03, 
                    link_frame_r="r_wrist_back_frame",dur=2.0):
                return FAILURE
            if not GripUtils.go_to_pts(point_l=self.location,grip_r=True, grip_l=True, point_r=self.location,
                    roll_l=pi/2,yaw_l=0,pitch_l=-pi/2,
                    y_offset_l=(self.distance+initial_separation)/2.0, z_offset_l=-0.03,
                    link_frame_l="l_wrist_back_frame",
                    roll_r=pi/2,yaw_r=0,pitch_r=-pi/2,
                    y_offset_r=-1*(self.distance+initial_separation)/2.0, z_offset_r=-0.03,
                    link_frame_r="r_wrist_back_frame",dur=2.0):
                return FAILURE
        else:
            #Right is negative in the y axis
            if self.arm=="r":
                y_multiplier = -1
            else:
                y_multiplier = 1
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    z_offset=0.05,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=4.0):
                return FAILURE
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    z_offset=-0.01,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=2.0):
                return FAILURE
            if not GripUtils.go_to_pt(point=self.location,grip=True,roll=pi/2,yaw=0,pitch=-pi/2,
                    y_offset=y_multiplier*self.distance,z_offset=-0.01,arm=self.arm,
                    link_frame="%s_wrist_back_frame"%self.arm,dur=2.0):
                return FAILURE
        return SUCCESS


class Sweep(SuccessFailureState):
    def __init__(self,direction,drag_amount,table_width,lift_amount):
        SuccessFailureState.__init__(self)
        self.direction = direction
        self.drag_amount = drag_amount
        self.table_width = table_width
        self.lift_amount = lift_amount
    def execute(self, userdata):
        forward_amount = 0.5
        hover_amount = 0.06
        if self.direction == "r":
            drag_arm = "r"
            edge_y = self.table_width / 2.0
            drag_yaw = pi/2
            final_y = edge_y - self.drag_amount
        else:
            drag_arm = "l"
            edge_y = -1 * self.table_width / 2.0
            drag_yaw = -pi/2
            final_y = edge_y + self.drag_amount

        if not GripUtils.go_to(x=forward_amount,y=edge_y,z=self.lift_amount,
                roll=0,pitch=0,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            return FAILURE
        if not GripUtils.go_to(x=forward_amount,y=edge_y,z=hover_amount,
                roll=0,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            return FAILURE

        if not GripUtils.go_to(x=forward_amount,y=final_y,z=hover_amount,
                roll=0,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            return FAILURE
        return SUCCESS

class SweepSameSide(SuccessFailureState):
    def __init__(self,direction,drag_amount,table_width,lift_amount):
        SuccessFailureState.__init__(self)
        self.direction = direction
        self.drag_amount = drag_amount
        self.table_width = table_width
        self.lift_amount = lift_amount
    def execute(self, userdata):
        forward_amount = 0.5
        hover_amount = 0.06
        if self.direction == "r":
            drag_arm = "r"
            edge_y = -1.0 * self.table_width / 2.0
            drag_yaw = pi/2
            final_y = edge_y + self.drag_amount
        else:
            drag_arm = "l"
            edge_y = 1.0 * self.table_width / 2.0
            drag_yaw = -pi/2
            final_y = edge_y - self.drag_amount

        if not GripUtils.go_to(x=forward_amount*.8,y=edge_y*1.25,z=self.lift_amount,
                roll=0,pitch=0,yaw=drag_yaw/4,grip=True,
                frame="table_height",arm=drag_arm):
            print 'f1'
            return FAILURE
        if not GripUtils.go_to(x=forward_amount,y=edge_y,z=hover_amount,
                roll=0,pitch=pi/4,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            print 'f2'
            return FAILURE

        if not GripUtils.go_to(x=forward_amount,y=final_y,z=hover_amount,
                roll=0,pitch=pi/2,yaw=drag_yaw,grip=True,
                frame="table_height",arm=drag_arm):
            print 'f3'
            return FAILURE
        return SUCCESS

class ShakeOneArm(SuccessFailureState):
    def __init__(self,arm,num_shakes):
        SuccessFailureState.__init__(self)
        self.arm = arm 
        if num_shakes < 1:
            rospy.logwarn("Must shake at least once.")
            num_shakes = 1
        self.num_shakes  = num_shakes

    def execute(self, userdata):

        forward_amount  = 0.375
        height          = 0.65
        drop_amount     = 0.25
        lateral_amount  = 0.05

        if self.arm=="r":
            yaw_multiplier = 1
            y_multiplier = -1
        else:
            yaw_multiplier = -1
            y_multiplier = 1
        
        for i in range(self.num_shakes):
            if i == 0:
                duration = 4.0
            if not GripUtils.go_to(x=forward_amount,y=0,z=height,
                    roll=0,pitch=0,yaw=yaw_multiplier*pi/2,grip=True,
                    frame="table_height",arm=self.arm,dur=duration):
                return FAILURE
            duration = .5
            if not GripUtils.go_to(x=forward_amount,y=lateral_amount*y_multiplier,z=height-drop_amount,
                    roll=0,pitch=pi/4,yaw=yaw_multiplier*pi/2,grip=True,
                    frame="table_height",arm=self.arm,dur=duration):
                return FAILURE
        if not GripUtils.go_to(x=forward_amount,y=0,z=height,
            roll=0,pitch=0,yaw=yaw_multiplier*pi/2,grip=True,
                frame="table_height",arm=self.arm,dur=duration):
            return FAILURE
        return SUCCESS

class ShakeBothArms(SuccessFailureState):
    def __init__(self,num_shakes, clothWidthScaleFactor=.85, input_keys=['cloth_width']):
        SuccessFailureState.__init__(self, input_keys=input_keys)
        if num_shakes < 1:
            rospy.logwarn("Must shake at least once.")
            num_shakes = 1
        self.num_shakes  = num_shakes   
        self.clothWidthScaleFactor = clothWidthScaleFactor

    def execute(self, userdata):

        cloth_width     = userdata.cloth_width

        if cloth_width < 0.35:
            rospy.logwarn("Cannot shake at less than 0.35m apart; tried to do %s" % str(cloth_width))
            return FAILURE
        
        if cloth_width > 0.65:
            cloth_width = 0.65

        forward_amount  = 0.45
        height          = 0.625
        drop_amount     = 0.3
        lateral_amount  = 0.1

        for i in range(self.num_shakes):
            if i == 0:
                duration = 4.0
            if not GripUtils.go_to_multi(x_l=forward_amount,y_l=cloth_width/2.0,z_l=height,
                                        roll_l=0,pitch_l=0,yaw_l=-pi/2,grip_l=True,
                                        x_r=forward_amount,y_r=-cloth_width/2.0,z_r=height,
                                        roll_r=0,pitch_r=0,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=duration):
                return FAILURE

            duration = .45
            return_val = SUCCESS
            if not GripUtils.go_to_multi(x_l=forward_amount,y_l=cloth_width/2.0-lateral_amount,z_l=height-drop_amount,
                                        roll_l=0,pitch_l=pi/4,yaw_l=-pi/2,grip_l=True,
                                        x_r=forward_amount,y_r=-cloth_width/2.0+lateral_amount,z_r=height-drop_amount,
                                        roll_r=0,pitch_r=pi/4,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=duration):
                return_val = FAILURE
        if not GripUtils.go_to_multi(x_l=forward_amount,y_l=cloth_width/2.0,z_l=height,
                                        roll_l=0,pitch_l=0,yaw_l=-pi/2,grip_l=True,
                                        x_r=forward_amount,y_r=-cloth_width/2.0,z_r=height,
                                        roll_r=0,pitch_r=0,yaw_r=pi/2,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=duration):
            return_val = FAILURE

        cloth_width = cloth_width * self.clothWidthScaleFactor
        if not GripUtils.go_to_multi(x_l=forward_amount+.2,y_l=cloth_width/2.0,z_l=height,
                                        roll_l=0,pitch_l=0,yaw_l=-pi/4,grip_l=True,
                                        x_r=forward_amount+.2,y_r=-cloth_width/2.0,z_r=height,
                                        roll_r=0,pitch_r=0,yaw_r=pi/4,grip_r=True,
                                        frame_l="table_height",frame_r="table_height",dur=4.0):
            return FAILURE

        return SUCCESS

class ArmsUp(SuccessFailureState):
    def execute(self, userdata):
        height = 0.35
        lateral_amount = 0.65
        forward_amount = 0.3

        if not GripUtils.go_to_multi(   x_l=forward_amount, y_l=lateral_amount, z_l=height,
                                        roll_l=0, pitch_l=0, yaw_l=0, grip_l=True,
                                        x_r=forward_amount, y_r=-lateral_amount, z_r=height,
                                        roll_r=0, pitch_r=0, yaw_r=0, grip_r=True,
                                        frame_l="torso_lift_link", frame_r="torso_lift_link", dur=4.0):
            return FAILURE
        else:
            return SUCCESS

class RecallArm(SuccessFailureState):
    def __init__(self, arm):
        SuccessFailureState.__init__(self)
        self.arm = arm
    def execute(self, userdata):

        if GripUtils.recall_arm(self.arm):
            return SUCCESS
        else:
            return FAILURE

class OpenGrippers(SuccessFailureState):
    def __init__(self, arm='b'):
        SuccessFailureState.__init__(self)
        self.arm = arm
    def execute(self, userdata):
        if not GripUtils.open_gripper(self.arm):
            return FAILURE
        else:
            return SUCCESS

class CloseGrippers(SuccessFailureState):
    def __init__(self, arm='b'):
        self.arm = arm
    def execute(self, userdata):
        if not GripUtils.close_gripper(self.arm):
            return FAILURE
        else:
            return SUCCESS
