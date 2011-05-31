#include <ros/ros.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_arm_kinematics/pr2_arm_ik_solver.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <urdf/model.h>

namespace pr2_simple_arm_motions{

    class LegalChecker{

        static const std::string ik_seed_name_left_[];
        static const std::string ik_seed_name_right_[];
        static const double ik_seed_position_left_[];   
        static const double ik_seed_position_right_[];

        public:

        explicit LegalChecker(ros::NodeHandle nh_toplevel);
        ~LegalChecker(void);        
        bool isLegal(geometry_msgs::PointStamped goal_point, double roll, double pitch, double yaw, char arm, const std::string& link_frame);
        bool isLegal(geometry_msgs::PointStamped goal_point, double roll, double pitch, double yaw, char arm);
        bool isLegal(geometry_msgs::PoseStamped goal_pose, char arm);
        int getJointIndex(const std::string &name, char arm);


        protected:

        ros::NodeHandle nh_;
        urdf::Model robot_model_;
        int dimension_, free_angle_;
        double search_discretization_, timeout_;
        std::string root_name_, tip_name_left_, tip_name_right_;

        KDL::Chain kdl_chain_left_, kdl_chain_right_;
        KDL::JntArray jnt_pos_suggestion_left_, jnt_pos_suggestion_right_;

        boost::shared_ptr<pr2_arm_kinematics::PR2ArmIKSolver> pr2_arm_ik_solver_left_,pr2_arm_ik_solver_right_;
        tf::TransformListener tf_;

        tf::StampedTransform l_tip_to_wrist_;
        tf::StampedTransform l_grip_to_wrist_;
        tf::StampedTransform r_tip_to_wrist_;
        tf::StampedTransform r_grip_to_wrist_;


    };
}
