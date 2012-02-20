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
import operator
from rll_utils import TFUtils

"""
Better documentation to come

Provides some simple services for moving the base around

Services:

---
set_pose
---
Move the base to the specified pose. WARNING: DOES NOT CHECK FOR COLLISIONS!

---
move_base
---
Move the base by a relas move_all_maptive dx,dy amount

---
found_box
---
Interfaces with the box_detector to say whether or not you've gotten within range of a box. Used to keep the robot from hitting the table

"""

class BaseMover:
    def __init__(self, start_srvs=True):
        self.listener = TFUtils.SimpleTransformListener()
        self.found_box = False
        self.using_checkerboard = rospy.get_param('~using_checkerboard',False)
        self.world_frame = rospy.get_param('~world_frame','odom_combined')
        if start_srvs:
            self.base_move_srv = rospy.Service('move_base', MoveBase, self.move_base_srv)
            self.base_rotate_srv = rospy.Service('rotate_base', RotateBase, self.rotate_base)
            self.set_pose_service = rospy.Service('set_pose', MoveBaseToPose, self.set_pose_srv)
        self.base_pub = rospy.Publisher("base_controller/command",Twist)

    def set_pose_srv(self, req):
        self.set_pose(req.target)

    def set_pose(self,target):
        rospy.loginfo("Received new base pose request")
        kp = [0.7, 0.7, 1]
        kd = 0
        ki = [0.1, 0.1, 0.5] #angular impatience is greater

        #Notation:
        #le = lateral error (displacement)
        #re = rotational error (angle)
        #d = derivative
        #i = integral
        goal = target

        i_le = [0, 0, 0] # i_le = approximate integral of lateral error

        first_time = True

        while(True):
            #P term
            le = list(self.goal_pose_displ(goal))
            p = [le[i]*kp[i] for i in range(len(le))]

            if first_time:
                prev_le = le
                first_time = False

            #D term
            d_le = [x/0.01 for x in map(operator.sub, le, prev_le)] #d_e_t = derivative of error
            d = [x*kd for x in d_le]

            #I term
            d_i_le = [x*0.01 for x in le] #d_i_le = change in integral of error (by Rectangle method)
            i_le = [i_le[i] + d_i_le[i] for i in range(len(i_le))]
            i = [i_le[i]*ki[i] for i in range(len(i_le))]

            prev_le = le

            rospy.loginfo("Still have to go: %s", le)
            if self.magnitude(le[0:2]) < 0.05 and le[2] < math.pi/72: #TODO: Tune this
                break
            command = Twist()
            command.linear.x = p[0] + d[0]+ i[0]
            command.linear.y = p[1] + d[1]+ i[1]
            command.angular.z = p[2] + d[2] + i[2]
            self.base_pub.publish(command)
            rospy.sleep(0.01)
        self.base_pub.publish(Twist()) #Stops

        # Try and prevent under-shooting
        self.move_base(le[0], le[1])

        return True
        
    def goal_pose_displ(self,goal):
        now = rospy.Time.now()
        self.listener.waitForTransform("base_footprint",goal.header.frame_id,now,rospy.Duration(20.0))
        goal.header.stamp = now #would this screw up localization? we want that location in the frame in the past, don't we?
        displ = self.listener.transformPose("base_footprint",goal)
        
        x_displ = displ.pose.position.x
        y_displ = displ.pose.position.y
        angle = sign(displ.pose.orientation.z)*2*math.acos(displ.pose.orientation.w)

        return (x_displ, y_displ, angle)

    def magnitude(self, vector):
        sum = 0
        for component in vector:
            sum += component ** 2
        return math.sqrt(sum)

    def move_base_srv(self, req):
        self.move_base(req.x, req.y)

    def move_base(self,dx,dy):
        EPS = 0.03

        # Typically seems to under-shoot for smaller base movements
        if abs(dx) <= 0.2 and dx != 0:
            dx += dx/abs(dx)*EPS
        if abs(dy) <= 0.2 and dy != 0:
            dy += dy/abs(dy)*EPS

        rospy.loginfo("Received new base displacement request")                 
        kp = 0.7
        kd = 0
        ki = 0.1

        goal = self.goal(dx, dy)
        prev_e_t = list(self.goal_displ(goal))
        i_e_t = [0, 0] # i_e_t = approximate integral of error

        first_time = True

        while(True):
            #P term
            e_t = list(self.goal_displ(goal)) #e_t = e(t) = current error
            p = [x*kp for x in e_t]

            if first_time:
                prev_e_t = e_t
                first_time = False

            #D term
            d_e_t = [x/0.01 for x in map(operator.sub, e_t, prev_e_t)] #d_e_t = derivative of error
            d = [x*kd for x in d_e_t]

            #I term
            d_i_e_t = [x*0.01 for x in e_t] #d_i_e_t = change in integral of error (by Rectangle method)
            i_e_t = [i_e_t[i] + d_i_e_t[i] for i in range(len(i_e_t))]
            i = [x*ki for x in i_e_t]

            prev_e_t = e_t

            print "Still have to go: ", e_t
            if self.magnitude(e_t) < EPS:
                break
            command = Twist()
            command.linear.x = p[0] + d[0] + i[0]
            command.linear.y = p[1] + d[1] + i[1]
            self.base_pub.publish(command)
            rospy.sleep(0.01)
        self.base_pub.publish(Twist()) #Stops
        return True

    def sign(self, num):
        if num > 0:
            return 1
        elif num < 0:
            return -1
        else:
            return 0

    def normalize(self, xytuple):
        (x, y) = xytuple
        x_comp = math.fabs((x/y)/math.sqrt(1+((x/y) ** 2))) * self.sign(x)
        y_comp = math.fabs((y/x)/math.sqrt(1+((y/x) ** 2))) * self.sign(y)
        return (x_comp, y_comp)

    def rotate_base_srv(self, req):
        self.move_base(req.yaw)

    def rotate_base(yaw):

        #GOTTA TAKE SIGN OF TEH ANGLE INTO ACCOUNT!
        while(yaw > math.pi):
            yaw -= 2*math.pi
        while(yaw <= -1*math.pi):
            yaw += 2*math.pi
        if yaw < 0:
            sign = -1
        else:
            sign = 1

        rospy.loginfo("Received new base rotation request")
        kp = 1
        kd = 0
        ki = 0.5

        rospy.loginfo("Getting goal")

        goal = self.rotate_goal(yaw)

        rospy.loginfo("Got goal")

        prev_e_t = self.rotate_goal_displ(goal)

        rospy.loginfo("Got displacement.")

        i_e_t = 0 # i_e_t = approximate integral of error

        first_time = True
        below_threshold = [False, False, False] #whether was below the threshold in the last 3 timesteps

        while(True):
            #P term
            e_t = self.rotate_goal_displ(goal) #e_t = e(t) = current error (limited to be between -pi and pi)
            while e_t > math.pi:
                e_t -= 2*math.pi
            while e_t <= -math.pi:
                e_t += 2*math.pi
            p = e_t*kp
            
            if first_time:
                prev_e_t = e_t
                first_time = False

            #D term
            d_e_t = e_t - prev_e_t #d_e_t = derivative of error
            d = d_e_t*kd

            #I term
            d_i_e_t = e_t*0.01 #d_i_e_t = change in integral of error (by Rectangle method)
            i_e_t = i_e_t + d_i_e_t
            i = i_e_t*ki

            prev_e_t = e_t

            rospy.loginfo("Still have to go: %s radians.", sign*e_t)
            below_threshold.pop(0) #remove first item
            if math.fabs(e_t) < 0.05:
                below_threshold.append(True)
            else:
                below_threshold.append(False)

            if reduce(lambda x, y: x and y, below_threshold):
                break

            command = Twist()
            command.angular.z = p+d+i
            self.base_pub.publish(command)
            rospy.sleep(0.01)
        self.base_pub.publish(Twist()) #Stops
        return True

    def current_position(self):
        origin = PointStamped()
        origin.header.frame_id = "base_footprint"
        now = rospy.Time.now()
        self.listener.waitForTransform(self.world_frame,"base_footprint",rospy.Time.now(),rospy.Duration(2.0))
        origin.header.stamp = now
        current_point = self.listener.transformPoint(self.world_frame,origin)
        return (current_point.point.x,current_point.point.y)
   
    def goal(self,x,y):
        goal = PointStamped()
        goal.header.frame_id = "base_footprint"    
        goal.point.x = x
        goal.point.y = y
        goal_point = self.listener.transformPoint(self.world_frame,goal)
        return goal_point
        
    def goal_displ(self,orig_goal):
        displ = self.listener.transformPoint("base_footprint",orig_goal)
        return (displ.point.x,displ.point.y)

    def rotate_goal(self,yaw):
        goal = PoseStamped()
        goal.header.frame_id = "base_footprint"

        goal.pose.orientation.z = sin(yaw/2)
        #using the commented version makes negative angles not work (does the positive angle instead)
        #goal.pose.orientation.z = sign(yaw)*sin(yaw/2)
        goal.pose.orientation.w = cos(yaw/2)

        goal_point = self.listener.transformPose(self.world_frame,goal)
        return goal_point
        
    def rotate_goal_displ(self,orig_goal):
        displ = self.listener.transformPose("base_footprint",orig_goal)
        
        angle = sign(displ.pose.orientation.z)*2*math.acos(displ.pose.orientation.w)
        return angle

def main(args):
    rospy.init_node("base_mover")
    bm = BaseMover()
    rospy.spin()

if __name__ == '__main__':
    args = sys.argv[1:]
    try:
        main(args)
    except rospy.ROSInterruptException: pass

