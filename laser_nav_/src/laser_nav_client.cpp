//#include <boost/thread/mutex.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
//#include <ros/ros.h>
//#include <boost/bind.hpp>
// move_base client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// explore action server
#include <actionlib/server/simple_action_server.h>
#include "laser_nav_/ExploreAction.h"
using namespace std;
typedef actionlib::SimpleActionServer<laser_nav_::ExploreAction> Server;
class action
{
    public:
    action();
    
    private:
    ros::NodeHandle nl;
    tf::TransformListener tf;
    actionlib::SimpleActionServer<laser_nav_::ExploreAction> server;
    // Client for move_base
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    move_base_msgs::MoveBaseGoal move_client_goal_;
    //void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    //void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    boost::mutex move_client_lock_;

    void execute(const laser_nav_::ExploreGoalConstPtr &goal);
    void preemptCb();
};
action::action():
    move_client_("move_base",true),
    tf(ros::Duration(10.0)),
    server(nl,"exploration",boost::bind(&action::execute,this,_1),false)
{
    server.registerPreemptCallback(boost::bind(&action::preemptCb, this));
    server.start();
}
void action::execute(const laser_nav_::ExploreGoalConstPtr &goal)
{
    
}
void action::preemptCb()//取消任务
{
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    ROS_WARN("Current exploration task cancelled");

    if(server.isActive()){
        server.setPreempted();
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration");
    action action;
    ros::spin();
    return 0;
}