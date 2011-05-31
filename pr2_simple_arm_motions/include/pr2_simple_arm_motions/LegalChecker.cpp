
#include <pr2_simple_arm_motions/LegalChecker.h>
using namespace pr2_simple_arm_motions;

LegalChecker::LegalChecker(ros::NodeHandle nh_toplevel){
  dimension_ = 7;
  // Load Robot Model
  ROS_INFO("Loading robot model");
  std::string xml_string;
  if (!nh_toplevel.getParam(std::string("/robot_description"), xml_string))
  {
    ROS_ERROR("Could not find parameter robot_description on parameter server.");
    ros::shutdown();
    exit(1);
  }
  if(!robot_model_.initString(xml_string))
  {
    ROS_ERROR("Could not load robot model.");
    ros::shutdown();
    exit(1);
  }

  // Get Parameters 
  nh_.param("free_angle", free_angle_, 2);
  nh_.param("search_discretization", search_discretization_, 0.01);
  nh_.param("ik_timeout", timeout_, 5.0);
  root_name_ = "torso_lift_link";
  tip_name_left_ = "l_wrist_roll_link"; 
  tip_name_right_ = "r_wrist_roll_link"; 

  // Init pose suggestion
  jnt_pos_suggestion_left_.resize(dimension_);
  jnt_pos_suggestion_right_.resize(dimension_);

  ROS_INFO("Loading KDL chain");
  KDL::Tree tree_left, tree_right;
  if (!kdl_parser::treeFromUrdfModel(robot_model_, tree_left))
  {
    ROS_ERROR("Could not load the KDL tree from the robot model");
    ros::shutdown();
    exit(1);
  }
  if (!tree_left.getChain(root_name_, tip_name_left_, kdl_chain_left_))
  {
    ROS_ERROR("Could not create the left KDL chain");
    ros::shutdown();
    exit(1);
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model_, tree_right))
  {
    ROS_ERROR("Could not load the KDL tree from the robot model");
    ros::shutdown();
    exit(1);
  }
  if (!tree_right.getChain(root_name_, tip_name_right_, kdl_chain_right_))
  {
    ROS_ERROR("Could not create the left KDL chain");
    ros::shutdown();
    exit(1);
  }

  // Init IK
  ROS_DEBUG("Starting with search_discretization %f and ik_timeout %f", search_discretization_,timeout_);
#include <geometry_msgs/PoseStamped.h>
  pr2_arm_ik_solver_left_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model_, root_name_, tip_name_left_, search_discretization_, free_angle_));
  pr2_arm_ik_solver_right_.reset(new pr2_arm_kinematics::PR2ArmIKSolver(robot_model_, root_name_, tip_name_right_, search_discretization_, free_angle_));

  if(!pr2_arm_ik_solver_left_->active_)
  {
    ROS_ERROR("Could not load left pr2 arm IK solver");
    ros::shutdown();
    exit(1);
  }
  if(!pr2_arm_ik_solver_right_->active_)
  {
    ROS_ERROR("Could not load right pr2 arm IK solver");
    ros::shutdown();
    exit(1);
  }

  tf_.lookupTransform("/l_wrist_roll_link",  "/l_grip_frame",  ros::Time(0), l_grip_to_wrist_);
  tf_.lookupTransform("/l_wrist_roll_link",  "/l_tip_frame",   ros::Time(0),  l_tip_to_wrist_);
  tf_.lookupTransform("/r_wrist_roll_link",  "/r_grip_frame",  ros::Time(0), r_grip_to_wrist_);
  tf_.lookupTransform("/r_wrist_roll_link",  "/r_tip_frame",   ros::Time(0),  r_tip_to_wrist_);


}

LegalChecker::~LegalChecker(void){

};

bool LegalChecker::isLegal(geometry_msgs::PointStamped goal_point, double roll, double pitch, double yaw, char arm, const std::string& link_frame)
{

    tf::Point offset;

    if(link_frame == "l_tip_frame"){
        offset = l_tip_to_wrist_.getOrigin();
    } else if(link_frame == "r_tip_frame"){
        offset = r_tip_to_wrist_.getOrigin();
    } else if(link_frame == "l_grip_frame"){
        offset = l_grip_to_wrist_.getOrigin();
    } else if(link_frame == "r_grip_frame"){
        offset = r_grip_to_wrist_.getOrigin();
    } else {
        ROS_WARN("Using unsupported frame %s", link_frame.c_str());
        return false;
    }

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    tf::Transform q_transform(q);
    tf::Point offset_transformed = q_transform * offset;

    goal_point.point.x -= offset_transformed.x();
    goal_point.point.y -= offset_transformed.y();
    goal_point.point.z -= offset_transformed.z();

    return isLegal(goal_point, roll, pitch, yaw, arm);


}

bool LegalChecker::isLegal(geometry_msgs::PointStamped goal_point, double roll, double pitch, double yaw, char arm){

  //Point + RPY to pose
  KDL::Rotation rot = KDL::Rotation::RPY(roll, pitch, yaw);
  double x,y,z,w;
  rot.GetQuaternion(x,y,z,w);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.stamp = goal_point.header.stamp;
  goal_pose.header.frame_id = goal_point.header.frame_id;
  goal_pose.pose.position.x = goal_point.point.x;
  goal_pose.pose.position.y = goal_point.point.y;
  goal_pose.pose.position.z = goal_point.point.z;
  goal_pose.pose.orientation.x = x;
  goal_pose.pose.orientation.y = y;
  goal_pose.pose.orientation.z = z;
  goal_pose.pose.orientation.w = w;
  return isLegal(goal_pose, arm);
}

bool LegalChecker::isLegal(geometry_msgs::PoseStamped goal_pose, char arm){
  //Initial seeds
  bool ret1 = false;


  tf::Stamped<tf::Pose> tf_pose_stamped;
  KDL::Frame desired_pose;
  //Do the transform
  try
  {
    std::string error_msg;
    ros::Time now = ros::Time::now();
    ret1 = tf_.waitForTransform(root_name_, goal_pose.header.frame_id, now,
        ros::Duration(5.0), ros::Duration(0.01), &error_msg);
    // Transforms the pose into the root link frame
    goal_pose.header.stamp = now;
    tf::poseStampedMsgToTF(goal_pose, tf_pose_stamped);
    tf_.transformPose(root_name_, tf_pose_stamped, tf_pose_stamped);
    tf::PoseTFToKDL(tf_pose_stamped, desired_pose);
  }
  catch(const tf::TransformException &ex)
  {
    ROS_ERROR("Transform failure (%d): %s", ret1, ex.what());
    return false;
  }
  const std::string* ik_seed_name;
  const double* ik_seed_position;
  boost::shared_ptr<pr2_arm_kinematics::PR2ArmIKSolver> pr2_arm_ik_solver;
  KDL::JntArray jnt_pos_suggestion;
  
  if(arm == 'l')
  {
    ik_seed_name = ik_seed_name_left_;
    ik_seed_position = ik_seed_position_left_;
    pr2_arm_ik_solver = pr2_arm_ik_solver_left_;
    jnt_pos_suggestion = jnt_pos_suggestion_left_;
  }
  else
  {
    ik_seed_name = ik_seed_name_right_;
    ik_seed_position = ik_seed_position_right_;
    pr2_arm_ik_solver = pr2_arm_ik_solver_right_;
    jnt_pos_suggestion = jnt_pos_suggestion_right_;
  }
  // Get the IK seed from the goal FIXME 
  for(int i=0; i < dimension_; i++)
  {
    jnt_pos_suggestion(getJointIndex(ik_seed_name[i],arm)) = ik_seed_position[i];
  }

  KDL::JntArray jnt_pos_out;
  bool is_valid = (pr2_arm_ik_solver->CartToJntSearch(jnt_pos_suggestion, desired_pose, jnt_pos_out, timeout_)>=0);
  return is_valid;
}

int LegalChecker::getJointIndex(const std::string &name, char arm)
{
  int i=0; // segment number
  int j=0; // joint number
  KDL::Chain kdl_chain;
  if(arm == 'l')
  {
    kdl_chain = kdl_chain_left_;
  }
  else
  {
    kdl_chain = kdl_chain_right_;
  }
  while(j < dimension_ && i < (int) kdl_chain.getNrOfSegments())
  {
    if(kdl_chain.getSegment(i).getJoint().getType() == KDL::Joint::None)
    {
      i++;
      continue;
    }
    if(kdl_chain.getSegment(i).getJoint().getName() == name)
    {
      return j;
    }
    i++;
    j++;
  }
  return -1;
}

const std::string LegalChecker::ik_seed_name_left_[] =  {"l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", 
    "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", 
    "l_wrist_roll_joint"};
const std::string LegalChecker::ik_seed_name_right_[] = {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", 
    "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", 
    "r_wrist_roll_joint"};
const double LegalChecker::ik_seed_position_left_[] =   {0.83007285324949953,-0.35320448964307055,1.7675383848900958,
    -1.1557956529332751,1.5035863485216983,-1.4448739148710006, 
    -1.5598103297885944};   
const double LegalChecker::ik_seed_position_right_[] =  {-0.58382693133119901,-0.35320448964307055,-1.7535368129157851,
    -0.99857677756936158,-1.2030484613896466,-1.293617529636232,
    -1.3099584235715724};

