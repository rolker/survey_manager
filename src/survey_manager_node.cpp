#include "ros/ros.h"
#include "survey_manager/SonarCoverage.h"
//#include "kongsberg_em_control/EMControl.h"
#include "std_msgs/Int32.h"
#include "nodelet/loader.h"

void wptIndexCallback(const std_msgs::Int32::ConstPtr &message)
{
//     kongsberg_em_control::EMControl emc;
//     emc.request.line_number = -1;
//     emc.request.requested_mode = 2;
//     ros::service::call("/sonar/control",emc);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "survey_manager");
    ros::NodeHandle n;

    ros::Subscriber wptIndexSub = n.subscribe("/moos/wpt_index",10,&wptIndexCallback);
    
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "survey_manager/SonarCoverage", remap, nargv);
    
    ros::spin();
}
