#include "ExplorationPlannerROS/ExplorationPlannerROS.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "top_exploration_planner_node");
    ROS_INFO("Started node top_exploration_planner_node");
    explorationplanner_ros::ExplorationPlannerROS planner("exploration");

    ros::spin();
    return 0;
}
