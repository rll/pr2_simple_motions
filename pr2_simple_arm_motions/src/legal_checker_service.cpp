#include <iostream>
#include <ros/ros.h>
#include <pr2_simple_arm_motions/LegalChecker.h>
#include <pr2_simple_motions_srvs/IsLegal.h>

pr2_simple_arm_motions::LegalChecker* legalChecker;

bool is_legal_srv(  pr2_simple_motions_srvs::IsLegal::Request    &req,
                    pr2_simple_motions_srvs::IsLegal::Response   &res )
{
    std::string frame;
    if(req.link_frame == ""){
        frame = req.arm + "_tip_frame";
        std::cout << "using famre " << frame << std::endl;
    } else{
        frame = req.link_frame;
    }

    res.legal = legalChecker->isLegal(req.goal_point,req.roll,req.pitch,req.yaw,req.arm[0],frame);

    return true;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "is_legal_server");
    ros::NodeHandle n;
    legalChecker = new pr2_simple_arm_motions::LegalChecker(n);
    ros::ServiceServer service = n.advertiseService("is_legal", is_legal_srv);
    ROS_INFO("is_legal service ready.");
    ros::spin();
}
