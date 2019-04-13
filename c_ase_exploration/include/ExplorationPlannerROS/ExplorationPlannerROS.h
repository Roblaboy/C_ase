#ifndef EXPLORATIONPLANNERROS_H
#define EXPLORATIONPLANNERROS_H
#include <boost/thread/mutex.hpp>
#include <string>
#include <memory>
#include <cmath>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
// Map subscribing
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>

// move_base client
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// explore action server
#include <actionlib/server/simple_action_server.h>
#include "ase_exploration/ExploreAction.h"
#include "ExplorationPlanner/planner/SMCPlanner.h"

// Dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <ase_exploration/PlannerConfig.h>

#include <visualization_msgs/Marker.h>
namespace explorationplanner_ros {

struct PlannerParamsROS
{
    bool wait_between_planner_iterations;

    double lin_v_min;
    double lin_v_max;
    double ang_v_min;
    double ang_v_max;
    double f_rot_min;
    double f_rot_max;

    // trajectory generation specific parameters
    double min_traj_length;
    double min_reward;
    int max_sampling_tries;

    double laser_min_angle_deg;
    double laser_max_angle_deg;
    double laser_angle_step_deg;
    double laser_max_dist_m;
    double p_false_pos;
    double p_false_neg;

    int horizon;
    double discount;

    int schedule_a;
    int schedule_b;
    int num_particles;
    double resample_thresh;

    int num_kernels;
    double std_vel;
    double std_ang;
    double std_fr;
    double default_ctrl_duration;

    bool allow_unknown_targets;

    double goal_execution_time;
    double frontier_distance_threshold;

    std::string map_frame_id_; // global map and costmap need to be in this frame
    std::string base_frame_id_;
};


class ExplorationPlannerROS
{
public:
    ExplorationPlannerROS(const std::string name);

private:
    // ROS connectivity
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf::TransformListener tf_listener_;
    ros::Subscriber top_goal_sub_;
    geometry_msgs::PoseStamped top_goal;
    bool top_succeed = false;
    void sent_goal(const geometry_msgs::PoseStamped &goal);

    
    // Server for exploration action
    void executeCb(const ase_exploration::ExploreGoalConstPtr &goal);
    void preemptCb();
    actionlib::SimpleActionServer<ase_exploration::ExploreAction> explore_server_;
    ase_exploration::ExploreFeedback feedback_;
    //laser_navigation
    int die_dai_cishu=0;
    int number=0;
    int disc=0;
    float Pi=3.1415926;
    float ori_cal(int a,int b);

    int start_n=0;
    int end_n=0;
    int safe_d=0.4;
    int safe_n=11;
    int ranges=686;
    int goal_id=0;
    float dx,dy=0.0;
    int j=1;
    float theta=0.0;
    float distance_to_goal=0.0;
    float min_distance=50.0;
    std::vector<float> x,y,x_map,y_map;
    geometry_msgs::PointStamped goal_pose_base;
    geometry_msgs::PointStamped goal_pose_map;
    geometry_msgs::Point p;
    float laser_data[686];
    bool start_laser_nav=true;
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    //ros::Publisher laser_goal_pub_;
    ros::Publisher ase_goal_pub_;
    ros::Publisher laser_goal_pub_;
    boost::mutex laser_data_lock_;
    ros::Publisher final_goal_pub;
    visualization_msgs::Marker marker;
    // Client for move_base
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_client_;
    move_base_msgs::MoveBaseGoal move_client_goal_;
    void doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    boost::mutex move_client_lock_;

    // subscribe to a costmap from move_base:
    // costmap is used to check validity of trajectories,
    // we maintain it in a sparse format.
    ros::Subscriber costmap_sub_;
    ros::Subscriber costmap_update_sub_;
    //
    ros::Subscriber scan_sub_;

    boost::mutex costmap_lock_;
    void costmapCb(const nav_msgs::OccupancyGrid &map);
    void costmapUpdateCb(const map_msgs::OccupancyGridUpdate &map_update);
    boost::shared_ptr<PlanarGridBinaryMap> trajectorycheck_map_;

    // subscribe to a global map, e.g. from SLAM.
    // this map is used to plan and simulate observations.
    // we maintain it in sparse format.
    ros::Subscriber global_map_sub_;
    boost::mutex global_map_lock_;
    void globalmapCb(const nav_msgs::OccupancyGrid &map);
    PlanarGridOccupancyMap global_map_;

    // dynamic reconfigure
    void confCb(ase_exploration::PlannerConfig &config, uint32_t level);
    dynamic_reconfigure::Server<ase_exploration::PlannerConfig> conf_server_;
    dynamic_reconfigure::Server<ase_exploration::PlannerConfig>::CallbackType confCb_;

    // internal parameters
    PlannerParamsROS params_;
    void readParameters(); // reads from parameter server

    // state of exploration task
    bool moving_;
    bool possibly_stuck_;
    bool goal_is_frontier_;
    bool goal_is_laser_;
    ros::Time goal_time_;

    // tools used by the planner
    boost::shared_ptr<TrajectoryGenerator> trajgenerator_;
    boost::shared_ptr<TrajectoryEvaluator> trajevaluator_;
    void updateTrajectoryGenerator();
    void updateTrajectoryEvaluator();
    std::unique_ptr<SMCPlanner> buildPlanner() const;

    // publishing the paths considered by the planner at each iteration
    void publishPath(const std::vector<PlanarPose>& p);
    ros::Publisher path_pub_;
    
};

} // namespace explorationplanner_ros

#endif /* EXPLORATIONPLANNERROS_H */
