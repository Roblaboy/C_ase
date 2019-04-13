#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <boost/math/constants/constants.hpp>
#include <algorithm>
#include <visualization_msgs/Marker.h> 
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "ExplorationPlannerROS/ExplorationPlannerROS.h"
#include "ExplorationPlanner/trajectory/TrajectoryChecker.h"
#include "ExplorationPlanner/trajectory/TrajectoryGenerator.h"
#include "ExplorationPlanner/kinematics/RangeConstraint.h"
#include "ExplorationPlanner/kinematics/VelocityPlanarKinematics.h"
#include "ExplorationPlanner/sensor/LaserScanner2D.h"
#include "ExplorationPlanner/planner/LinearCoolingSchedule.h"
#include "ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerUniform.h"
#include "ExplorationPlanner/kinematics/PlanarRobotVelCmdSamplerIndependentGaussian.h"
#include "ExplorationPlanner/planner/SMCPlanner.h"

// Frontier exploration services
#include <frontier_exploration/GetNextFrontier.h>
#include <frontier_exploration/UpdateBoundaryPolygon.h>
//Laser Scan navigation
#include <laser_nav_/laser_nav_srv.h>
//#include <sensor_msgs/LaserScan.h>
//#include "std_msgs/String.h"
namespace explorationplanner_ros {

ExplorationPlannerROS::ExplorationPlannerROS(const std::string name)
    : nh_(""),
      private_nh_("~"),
      tf_listener_(ros::Duration(10.0)),
      move_client_("move_base",true),
      explore_server_(nh_, name, boost::bind(&ExplorationPlannerROS::executeCb, this, _1), false),
      moving_(false),
      possibly_stuck_(false),
      goal_is_frontier_(false),
      global_map_(PlanarPose(0.0, 0.0, 0.0), 0.05)
{
    // Read parameters and prepare the object for use accordingly
    readParameters();  //读取参数

    // at this point, we can update trajectory evaluator (it uses only laser parameters)
    updateTrajectoryEvaluator();  //更新轨迹评估

    // Subscribe to costmap from move_base and a global map from e.g. SLAM
    costmap_sub_ = nh_.subscribe("costmap", 1, &ExplorationPlannerROS::costmapCb, this);  //订阅costmap,并执行costmapCb
    costmap_update_sub_ = nh_.subscribe("costmap_updates", 1, &ExplorationPlannerROS::costmapUpdateCb, this);//订阅costmap_updates，并执行costmapUpdateCb
    global_map_sub_ = nh_.subscribe("map", 1, &ExplorationPlannerROS::globalmapCb, this);//订阅map，并执行globalmapCb
    scan_sub_=nh_.subscribe<sensor_msgs::LaserScan>("/scan",1,&ExplorationPlannerROS::scan_callback, this);//订阅scan话题
    
    // dynamic reconfiguration setup
    confCb_ = boost::bind(&ExplorationPlannerROS::confCb, this, _1, _2);
    conf_server_.setCallback(confCb_);  //动态重配置设置

    // Action server start
    explore_server_.registerPreemptCallback(boost::bind(&ExplorationPlannerROS::preemptCb, this));
    explore_server_.start(); //启动移动服务器

    // paths inspected by the planner
    path_pub_ = nh_.advertise<nav_msgs::Path>("planner_paths", 10, true);//发布planner_paths话题
    //laser_goal_pub_=nh_.advertise<geometry_msgs::PoseStamped>("laser_goal_pose", 1);
    laser_goal_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 10 );
    ase_goal_pub_=nh_.advertise<geometry_msgs::PoseStamped>("ase_goal_pose", 1);
}

void ExplorationPlannerROS::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
//    int a=0;
//    wait_finished=false;
    ros::Rate r(10);
    //boost::unique_lock<boost::mutex> laser_data_lock(laser_data_lock_);
    //boost::mutex::scoped_lock laser_data_lock(laser_data_lock_);
    int ranges=scan->ranges.size();
    for(int i=0;i<ranges;i++)
    {
        laser_data[i]=scan->ranges.at(i);
    }
    r.sleep();
//    ROS_INFO("topic ranges=%d",ranges);
    //laser_data_lock.unlock();
//    //ROS_INFO("%d",ranges);
//    for(int i=0;i<ranges;i++)
//    {
//        //ROS_INFO("i:%d j:%d",i,j);
//        if(scan->ranges.at(i)==5.0) 
//        {
//            start_n=i;//标记弧线起始激光位置
//            for(j=i+1;j<ranges;++j)
//            {
//                if(scan->ranges.at(j)==5.0&&j!=(ranges-1))
//                {
//                    ++disc;
//                }
//               else if(disc > 9) 
//                {
//                    end_n=j;//遇到第一个不连续点，并且在这之前已经有连续的12个点，标记此弧线末端激光位置
//                    ROS_INFO("i:%d j:%d",i,j);
//                    theta=ori_cal(i,j);
//                    x.push_back(3*sin(theta-Pi/4));
//                    y.push_back(-3*cos(theta-Pi/4));
//                    disc=0;
//                    i=j;
//                    break;//扫完一段边界，跳出这层for循环
//                }
///               else break;//遇到第一个不连续点，并且在这之前已经没有连续的12个点，跳出这层for循环，继续扫描下一段
//            }
//        }
//                    //if(fabs(laser_data.at(i)-laser_data.at(i+1)))
//    }
//
//
}
float ExplorationPlannerROS::ori_cal(int a,int b)
{
    float rad=0;
    float center_id=0.0;
    center_id=(b+a)/2;
    //float Pi=3.1415926;
    rad=0.35*center_id*Pi/180;
    ROS_INFO("rad:%f",0.35*center_id);
    return rad;
}


void ExplorationPlannerROS::readParameters()  //读取参数
{
    private_nh_.param("wait_between_planner_iterations", params_.wait_between_planner_iterations, false);//是否等待用户输入在每一次的路径规划中

    private_nh_.param("lin_v_min", params_.lin_v_min, 0.3);//最小线速度
    private_nh_.param("lin_v_max", params_.lin_v_max, 1.0);//最大线速度
    private_nh_.param("ang_v_min", params_.ang_v_min, -1.0);//最小角速度
    private_nh_.param("ang_v_max", params_.ang_v_max, 1.0);//最大角速度
    private_nh_.param("f_rot_min", params_.f_rot_min, -0.02);//minimum final rotation at the end of the trajectory of the robot in radians在机器人路径末端的最终旋转最小弧度值
    private_nh_.param("f_rot_max", params_.f_rot_max, 0.02);//maximum final rotation at the end of the trajectory of the robot in radians 

    private_nh_.param("min_traj_length", params_.min_traj_length, 0.5);//规划器考虑到能获取的最短路径的长度值，米
    private_nh_.param("min_reward", params_.min_reward, 50.0);//规划器考虑到发现未知栅格的最小数目
    private_nh_.param("max_sampling_tries", params_.max_sampling_tries, 30);//用于描述每一条路径的最大样本点，在放弃或者是未发现可行路径之前

    private_nh_.param("laser_min_angle_deg", params_.laser_min_angle_deg, -90.0);//激光照射最小范围角度
    private_nh_.param("laser_max_angle_deg", params_.laser_max_angle_deg, 90.0);//激光照射最大范围角度
    private_nh_.param("laser_angle_step_deg", params_.laser_angle_step_deg, 5.0);//激光射线间隔角度
    private_nh_.param("laser_max_dist_m", params_.laser_max_dist_m, 4.0);//激光最大的可测量距离
    private_nh_.param("p_false_pos", params_.p_false_pos, 0.05);//激光假正读取（把自由认为是障碍）的概率
    private_nh_.param("p_false_neg", params_.p_false_neg, 0.05);//激光假负读取（把障碍认为是自由）的概率
    private_nh_.param("horizon", params_.horizon, 3);//确定每个路径将由多少控制动作组成
    private_nh_.param("discount", params_.discount, 1.0);//当下和之后的奖励相对值，应该大于0,值越小表示更着重于立即获取的信息，1对应于立即获取信息与之后的信息增益没有任何相关性

    private_nh_.param("schedule_a", params_.schedule_a, 3);//迭代第i次的时候，规划器使用k=a*i+b个样本来评估每一条路径，该设置是这个公式中的a
    private_nh_.param("schedule_b", params_.schedule_b, 3);//迭代第i次的时候，规划器使用k=a*i+b个样本来评估每一条路径，该设置是这个公式中的b
    private_nh_.param("num_kernels", params_.num_kernels, 5);//指定规划器运行迭代的总次数

    private_nh_.param("num_particles", params_.num_particles, 10);//在迭代集上保持的路径数，数越大，更能得出一个更好的路径，同样的，增加计算负荷
    ROS_ASSERT(params_.num_particles>0);
    private_nh_.param("resample_thresh", params_.resample_thresh, 0.33);//对于有效路径数低于重采样数时将会触发该阈值（0到1之间）

    private_nh_.param("std_vel", params_.std_vel, 0.2);//通过定义高斯分布的标准偏差，来控制路径采样过程，在速度高斯分布中，第i次迭代的标准偏差为std_vel/(i+1)
    private_nh_.param("std_ang", params_.std_ang, 0.1);//弧度分布中同上
    private_nh_.param("std_fr", params_.std_fr, 0.02);//最后一次旋转同上
    private_nh_.param("default_ctrl_duration", params_.default_ctrl_duration, 1.0);//每个控制动作的持续时间，以秒为单位

    private_nh_.param("allow_unknown_targets", params_.allow_unknown_targets, true);//是否允许产生未知区域的目标

    private_nh_.param("goal_execution_time", params_.goal_execution_time, 10.0);//单个任物执行允许最大时间
    private_nh_.param("frontier_distance_threshold", params_.frontier_distance_threshold, 2.0);//如果探索目标是通过边界探索获取的，当机器人认为已经到达目标并且重新规划时的最小距离

    private_nh_.param<std::string>("map_frame_id", params_.map_frame_id_, "map");//用于路径规划的地图框架
    private_nh_.param<std::string>("base_frame_id", params_.base_frame_id_, "base_link");//机器人的基础框架
}

void ExplorationPlannerROS::updateTrajectoryGenerator()//更新轨迹产生器
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    // Update the constraints 更新约束
    RangeConstraint linvel_c(params_.lin_v_min, params_.lin_v_max);//线速度范围
    RangeConstraint angvel_c(params_.ang_v_min, params_.ang_v_max);//角速度范围
    RangeConstraint rot_c(params_.f_rot_min, params_.f_rot_max);//最终旋转角度范围
    VelocityPlanarKinematics kin(linvel_c, angvel_c, rot_c);//定义了一个对象 kin，实参linvel_c, angvel_c, rot_c

    // rebuild object with new trajectory checker
    boost::shared_ptr<TrajectoryChecker>
            chk( new TrajectoryChecker(trajectorycheck_map_, params_.allow_unknown_targets));//定义了一个对象TrajectoryChecker, 实参trajectorycheck_map_，params_.allow_unknown_targets

    // construct new trajectory generator object
    trajgenerator_ = boost::shared_ptr<TrajectoryGenerator>(
                new TrajectoryGenerator(kin,
                                        chk,
                                        params_.min_traj_length,
                                        params_.max_sampling_tries)
                );//定义一个对象TrajectoryGenerator，实参kinchk,params_.min_traj_length, params_.max_sampling_tries
}

void ExplorationPlannerROS::updateTrajectoryEvaluator()//更新轨迹评价
{
    LaserScanner2D laser( params_.laser_min_angle_deg*(boost::math::constants::pi<double>()/180),//角度换算为弧度，最小弧度
                          params_.laser_max_angle_deg*(boost::math::constants::pi<double>()/180),//最大弧度
                          params_.laser_angle_step_deg*(boost::math::constants::pi<double>()/180),//laser间隔
                          params_.laser_max_dist_m,//激光最大的可测量距离
                          params_.p_false_pos,//激光假正读取（把自由认为是障碍）的概率
                          params_.p_false_neg);//激光假负读取（把障碍认为是自由）的概率
    trajevaluator_ = boost::shared_ptr<TrajectoryEvaluator>( new TrajectoryEvaluator(laser) );//trajevaluator_指向TrajectoryEvaluator，实参laser
}


std::unique_ptr<SMCPlanner> ExplorationPlannerROS::buildPlanner() const
{
    ROS_DEBUG("Building planner object...");
    LinearCoolingSchedule schedule(params_.schedule_a, params_.schedule_b);//定义一个LinearCoolingSchedule类，对象schedule,实参params_.schedule_a,params_.schedule_b
    SMCPlannerParameters param(schedule, params_.num_particles, params_.resample_thresh);//定义一个SMCPlannerParameters类，对象param,实参schedule, params_.num_particles, params_.resample_thresh
    PlanarRobotVelCmdSamplerUniform firstKC(params_.default_ctrl_duration);//定义一个 PlanarRobotVelCmdSamplerUniform类，对象firstKC,实参params_.default_ctrl_duration
    param.addKernel( firstKC );//向kernels_（std::vector）加入firstKC，kernel重采样
    for (unsigned int i = 0; i < params_.num_kernels; ++i)
    {
        const double ls = params_.std_vel  / (i+1);
        const double as = params_.std_ang / (i+1);
        const double fs = params_.std_fr / (i+1);
        PlanarRobotVelCmdSamplerIndependentGaussian kg(ls, as, fs, params_.default_ctrl_duration);
        param.addKernel( kg );//把kg加入到向量kernels_末尾
    }
    std::unique_ptr<SMCPlanner> smc(new SMCPlanner(params_.horizon, params_.discount, trajgenerator_, trajevaluator_, param));//建立了一个SMCPlanner对象，smc是它的独立指针
    ROS_DEBUG("Planner object built");
    return smc;
}


void ExplorationPlannerROS::confCb(ase_exploration::PlannerConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request, level: %d", level);
    if (level == 0)
    {
        ROS_INFO("Planner parameters reconfigured");//重配置Planner参数
        params_.horizon = config.horizon;
        params_.discount = config.discount;
        params_.schedule_a = config.schedule_a;
        params_.schedule_b = config.schedule_b;
        params_.num_kernels = config.num_kernels;
        params_.std_vel = config.std_vel;
        params_.std_ang = config.std_ang;
        params_.std_fr = config.std_fr;
        params_.default_ctrl_duration = config.default_ctrl_duration;
        params_.num_particles = config.num_particles;
        params_.resample_thresh = config.resample_thresh;
    }
    else if (level == 1)
    {
        ROS_DEBUG("Sensor parameters reconfigured");//重配置传感器参数
        params_.laser_min_angle_deg = config.laser_min_angle_deg;
        params_.laser_max_angle_deg = config.laser_max_angle_deg;
        params_.laser_angle_step_deg = config.laser_angle_step_deg;
        params_.laser_max_dist_m = config.laser_max_dist_m;
        params_.p_false_pos = config.laser_p_false_pos;
        params_.p_false_neg = config.laser_p_false_neg;
        updateTrajectoryEvaluator();
    }
    else if ( level == 2)
    {
        ROS_DEBUG("Trajectory generation parameters reconfigured");//路径产生器参数重配置
        params_.allow_unknown_targets = config.allow_unknown_targets;
    }
    else
        ROS_INFO("Undefined reconfiguration request");
}

void ExplorationPlannerROS::executeCb(const ase_exploration::ExploreGoalConstPtr &goal)
{
    ROS_INFO("Received new exploration task");
    moving_ = false;
    possibly_stuck_ = false;
    goal_is_frontier_ = false;

    // wait for move_base to become available
    if(!move_client_.waitForServer() )
    {
        explore_server_.setAborted();
        return;
    }

    //loop until the task is finished or pre-empted 循环直到任务完成或
    while(ros::ok() && explore_server_.isActive())
    {
        // get robot's current pose 获取机器人位置
        tf::StampedTransform tf_map_to_robot_base;
        try
        {
            tf_listener_.lookupTransform(params_.map_frame_id_, params_.base_frame_id_,
                                     ros::Time(0), tf_map_to_robot_base);//存有params_.base_frame_id_坐标原点在params_.map_frame_id_坐标系下的位置信息
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("TF Error: %s",ex.what());
            ROS_ERROR("Exploration failed");
            explore_server_.setAborted();
            return;
        }
        PlanarPose robot_planar_pose(tf_map_to_robot_base.getOrigin().getX(),//获取机器人在map_frame_id_的x坐标
                                     tf_map_to_robot_base.getOrigin().getY(),
                                     tf::getYaw(tf_map_to_robot_base.getRotation()) );
        ROS_INFO("robot_planar_pose:X:%f  Y:%f",robot_planar_pose.getX(),robot_planar_pose.getY());
        // Update the trajectory generator for this round 更新此轮的轨迹生成器
        updateTrajectoryGenerator();
        std::unique_ptr<SMCPlanner> planner = buildPlanner();//planner->SMCPlanner(params_.horizon, params_.discount, trajgenerator_, trajevaluator_, param)

        // get map information to use in planning 获取地图信息用于路径规划
        global_map_lock_.lock();
        ExplorationTaskState s(robot_planar_pose, global_map_ );
        global_map_lock_.unlock();

        // plan for requested number of steps 规划所要求的步骤数
        std::vector<double> avgRewards, minRewards, maxRewards, pathDist;
        unsigned int planner_iteration = 0;
        die_dai_cishu++;
        ROS_INFO("all of iteration times:%d",die_dai_cishu);
        while ( (planner_iteration < params_.num_kernels) && explore_server_.isActive() )//开始迭代
        {
            ros::Time t0 = ros::Time::now();
            if ( !planner->iterate(s) )//iterate()具体函数见SMCPlanner.cpp，粒子更新成功返回true，iterate+1
                ROS_WARN("Planner iteration failed");
            ros::Time t1 = ros::Time::now();
            ROS_INFO("Planner iteration %d complete in %f s", planner_iteration, (t1-t0).toSec());

            // publish reward and path information 发布奖励和路径信息 
            ParticleSet ps = planner->getParticleSet();//定义ps为ParticleSet类，getParticleSet() return particleset_
            std::vector<TrajectoryValueHandler> rh = planner->getLatestRewardHandlers();//return latest_rewards_ rh(particleset_.getNumberOfParticles粒子数目，TrajectoryValueHandler(getDiscount()))
            std::stringstream ss, sr, smin, smax;
            for (unsigned int i = 0; i < ps.getNumberOfParticles(); ++i)
            {
                std::vector<PlanarPose> path = ps[i].getTrajectory();//return trajectory_
                publishPath(path);

                ss << ps[i].getWeight() << ", ";//return weight_
                smin << rh[i].getMinSumOfDiscRew() << ", ";//min = std::min(min, v.getSumOfDiscountedRewards(discount_)) getSumOfDiscountedRewards(discount_)见Trajectoryvalue.h DiscRew是计算导航代价 SumOf是和
                smax << rh[i].getMaxSumOfDiscRew() << ", ";
                sr << rh[i].getAvgSumOfDiscRew() << ", ";

                // at last iteration, get summary information to decide what to do next
                if ( i == params_.num_kernels )
                {
                    avgRewards.push_back( rh[i].getAvgSumOfDiscRew() );
                    minRewards.push_back( rh[i].getMinSumOfDiscRew() );
                    maxRewards.push_back( rh[i].getMaxSumOfDiscRew() );


                    std::vector<PlanarPose> path = ps[i].getTrajectory();
                    PlanarPose target = path.end()[-2];
                    if ( path.size() == 2) // start pose + one other pose
                        target = path.back();

                    double dx = robot_planar_pose.getX() - target.getX();
                    double dy = robot_planar_pose.getY() - target.getY();
                    pathDist.push_back( std::sqrt( dx*dx + dy*dy  )  );
                }
            }

            // Print particle information
            ROS_DEBUG("Particle weights:           %s", ss.str().c_str());
            ROS_DEBUG("Minimum discounted rewards: %s", smin.str().c_str());
            ROS_DEBUG("Maximum discounted rewards: %s", smax.str().c_str());
            ROS_DEBUG("Average discounted rewards: %s", sr.str().c_str());

            if ( params_.wait_between_planner_iterations && (planner_iteration != params_.num_kernels) )
            {
                ROS_INFO("Press button to continue");
                std::cin.ignore();
            }
            ++planner_iteration;
        }

        // get the best plan from the planner
        std::vector<PlanarPose> plan;
        planner->getPlan(plan);

        // heuristics for checking if we might be stuck
        if ( std::all_of( avgRewards.begin(), avgRewards.end(), [this](double r){return r < params_.min_reward;} ) ||
             std::all_of( pathDist.begin(), pathDist.end(), [this](double d){return d < this->params_.min_traj_length; }   )
             )
        {
            ROS_WARN("All rewards less than minimum of %f required or target closer than given threshold %f meters, planner may be stuck!", params_.min_reward, params_.min_traj_length);
            possibly_stuck_ = true;
        }


        // Construct the goal pose message to be filled and sent to move_base
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = params_.map_frame_id_;

        if (!possibly_stuck_)
        {
            // Publish whole path for best particle
            // Get the goal pose -- move to second to last pose on the path (if it exists)
            // end() is past the last element, -1 for last element, -2 for second-last
            PlanarPose target = plan.end()[-2];
            if ( plan.size() == 2) // start pose + one other pose (horizon == 1)
                target = plan.back();

            goal_pose.pose.position.x = target.getX();
            goal_pose.pose.position.y = target.getY();
            tf::Quaternion q;
            q.setRPY( 0, 0, target.getTheta() );
            tf::quaternionTFToMsg(q, goal_pose.pose.orientation);
            ase_goal_pub_.publish(goal_pose);
            ROS_INFO("The next goal: X:%f Y:%f",goal_pose.pose.position.x,goal_pose.pose.position.y);

            // this goal is not a frontier
            goal_is_frontier_ = false;
        }
        else
        {
            // stuck - try to get a new target from frontier exploration
            ROS_INFO("Attempting recovery from stuck position via frontier exploration");

            // create the costmap service clients
            ros::ServiceClient updateBoundaryPolygon =
                    nh_.serviceClient<frontier_exploration::UpdateBoundaryPolygon>
                    ("frontier_exploration/explore_costmap/explore_boundary/update_boundary_polygon");
            ros::ServiceClient getNextFrontier =
                    nh_.serviceClient<frontier_exploration::GetNextFrontier>
                    ("frontier_exploration/explore_costmap/explore_boundary/get_next_frontier");
    
            if (!updateBoundaryPolygon.waitForExistence() ||
                    !getNextFrontier.waitForExistence() )
            {
                ROS_ERROR("Services for frontier exploration not available; exploration task canceled");
                explore_server_.setAborted();//
                return;
            }

            // set unbounded for frontier_exploration
            frontier_exploration::UpdateBoundaryPolygon bndsrv;
            bndsrv.request.explore_boundary.header.frame_id = params_.map_frame_id_;
            if(updateBoundaryPolygon.call(bndsrv))
            {
                ROS_DEBUG("Region boundary set for frontier_exploration");
            }
            else
            {
                ROS_ERROR("Failed to set region boundary for frontier_exploration");
                explore_server_.setAborted();
                return;
            }

            // Service call to frontier_exploration
            frontier_exploration::GetNextFrontier srv;
            // give current pose as start for frontier search
            srv.request.start_pose.header.frame_id = params_.map_frame_id_;
            srv.request.start_pose.pose.position.x = robot_planar_pose.getX();
            srv.request.start_pose.pose.position.y = robot_planar_pose.getY();
            tf::quaternionTFToMsg(tf_map_to_robot_base.getRotation(), srv.request.start_pose.pose.orientation);

            if ( getNextFrontier.call(srv) )
            {
                ROS_INFO("Got next frontier from frontier_exploration");
                goal_pose = srv.response.next_frontier;//
		        ROS_INFO("The next frontier: X:%f Y:%f",goal_pose.pose.position.x,goal_pose.pose.position.y);
            }
            else
            {
                ROS_ERROR("Call to service %s failed", getNextFrontier.getService().c_str() );
                //explore_server_.setAborted();
                //break;
                start_laser_nav=true;
            }
            // this goal is a frontier
            goal_is_frontier_ = true;
            possibly_stuck_ = false;
        }

        //start laser_navigation
        if(start_laser_nav)
        {
            //ros::ServiceClient laser_nav = 
                    //nh_.serviceClient<laser_nav_::laser_nav_srv>("select_laser_nav_goal");
            //service call to select_laser_nav_goal
            //laser_nav_::laser_nav_srv laser_srv;
            // give current pose based on robot's frame_id to laser navigation
            //laser_srv.request.base.header.frame_id = params_.base_frame_id_;
            //laser_srv.request.base.pose.position.x = 0;
            //laser_srv.request.base.pose.position.y = 0;
            //ROS_INFO("Robot_pose: X:%f Y:%f",robot_planar_pose.getX(),robot_planar_pose.getY());
            //tf::quaternionTFToMsg(tf_map_to_robot_base.getRotation(), laser_srv.request.robot_pose.pose.orientation);
            //ROS_INFO("Robot_pose_orientation: X:%f Y:%f Z:%f W:%f",
            //give ase_exploration goal position to laser navigation
            //laser_srv.request.goal_pose.header.frame_id = params_.map_frame_id_;
            //laser_srv.request.goal_pose.pose.position.x = goal_pose.pose.position.x;
            //laser_srv.request.goal_pose.pose.position.y = goal_pose.pose.position.y;
            //laser_srv.request.goal_pose.pose.orientation = goal_pose.pose.orientation ;
            //give laser scan data to laser navigation
            //for(int i=0;i<laser_data.size();++i)
            //{
            //    laser_srv.request.laser_data.push_back(laser_data.at(i));
            //}
            //if(laser_nav.call(laser_srv))
            //{
            //    ROS_INFO("Got next goal from laser_navigation");
            //    goal_pose = laser_srv.response.laser_goal_pose;
            //    ROS_WARN("X:%f Y:%f",goal_pose.pose.position.x,goal_pose.pose.position.y);
            //}
            //else
            //{
            //    ROS_ERROR("Call to service %s failed", laser_nav.getService().c_str() );
            //    explore_server_.setAborted();
            //    break;
            //}

                

                //boost::mutex::scoped_lock laser_data_lock(laser_data_lock_);
                //boost::unique_lock<boost::mutex> laser_data_lock(laser_data_lock_);
                
                //laser_data_lock.unlock();
                visualization_msgs::Marker marker;
                marker.header.frame_id = params_.map_frame_id_;
                marker.header.stamp = ros::Time(0);
                marker.ns="laser_goal";
                marker.id=0;
                marker.type = visualization_msgs::Marker::POINTS;
                marker.color.g=1.0;//green
                marker.color.a=1.0;
                marker.color.r = 0.0;
                marker.color.b = 0.0;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w=1.0;
                marker.scale.x = 0.2;  
                marker.scale.y = 0.2;
                marker.scale.z = 0;
                for(int i=1;i<ranges;i++)
                {
                //ROS_INFO("i:%d j:%d",i,j);
                    if(laser_data[i]>=4.0) 
                    {
                        start_n=i;//标记弧线起始激光位置
                        for(j=i+1;j<ranges;++j)
                        {
                            if(laser_data[j]>=4.0&&j!=(ranges-1))
                            {
                                ++disc;
                            }
                            else if(disc > 13) 
                            {
                                end_n=j;//遇到第一个不连续点，并且在这之前已经有连续的14个点，标记此弧线末端激光位置
                                ROS_INFO("i:%d j:%d",i,j);
                                theta=ori_cal(i,j);
                                x.push_back(3*sin(theta-Pi/6));
                                y.push_back(-3*cos(theta-Pi/6));                   
                                disc=0;
                                i=j;
                                break;//扫完一段边界，跳出这层for循环
                            }
                            else break;//遇到第一个不连续点，并且在这之前已经没有连续的14个点，跳出这层for循环，继续扫描下一段
                        }
                    }
                        if(fabs(laser_data[i]-laser_data[i-1])>=1.5)
                        {
                            if((laser_data[i]<=4.0)&&(laser_data[i-1]<=4.0))
                            {
                                if(laser_data[i-1]-laser_data[i]>0)
                                {
                                    theta=ori_cal(i-1,i-1)-0.3/laser_data[i];
                                    x.push_back(0.5*(laser_data[i-1]+laser_data[i])*sin(theta-Pi/4));
                                    y.push_back(-0.5*(laser_data[i-1]+laser_data[i])*cos(theta-Pi/4));
                                }
                                else
                                {
                                    theta=ori_cal(i-1,i-1)+0.3/laser_data[i];
                                    x.push_back(0.5*(laser_data[i-1]+laser_data[i])*sin(theta-Pi/4));
                                    y.push_back(-0.5*(laser_data[i-1]+laser_data[i])*cos(theta-Pi/4));
                                }
                            }
                        }
                }   
                for(int i=0;i<x.size();++i)
                {
                    ROS_INFO("vector: X:%f Y:%f",x.at(i),y.at(i));
                }
                if(!x.empty())
                {
                    for(int i=0;i<x.size();++i)
                    {
                        goal_pose_base.header.frame_id=params_.base_frame_id_;
                        goal_pose_base.point.x=x.at(i);
                        goal_pose_base.point.y=y.at(i);
                        goal_pose_base.point.z=0;
                        goal_pose_base.header.stamp=ros::Time(0);
                        try
                        {
                            tf_listener_.transformPoint(params_.map_frame_id_, goal_pose_base, goal_pose_map);
                        }
                        catch (tf::TransformException &ex) 
                        {
                            ROS_ERROR("%s",ex.what());
                            ros::Duration(1.0).sleep();
                        }
                        x_map.push_back(goal_pose_map.point.x);
                        y_map.push_back(goal_pose_map.point.y);
                    }     
                    for(int i=0;i<x_map.size();++i)  
                    {
                        //goal_pose.pose.position.x=x_map.at(i);
                        //goal_pose.pose.position.y=y_map.at(i); 
                        p.x=x_map.at(i);
                        p.y=y_map.at(i);
                        p.z=0.0;
                        marker.points.push_back(p);    
                        ROS_INFO("The next kexuan laser_goal: X:%f Y:%f",goal_pose.pose.position.x,goal_pose.pose.position.y);
                        dx=x_map.at(i)-goal_pose.pose.position.x;
                        dy=y_map.at(i)-goal_pose.pose.position.y;
                        distance_to_goal = std::sqrt( dx*dx + dy*dy );
                        ROS_INFO("distance_to_goal: %f",distance_to_goal);
                        if(min_distance>distance_to_goal) 
                        {
                            min_distance=distance_to_goal;
                            goal_id=i;   
                        }
                    }
                    ROS_INFO("min_distance: %f",min_distance);
                    laser_goal_pub_.publish(marker);
                    min_distance=50.0;
                    x.clear();
                    y.clear();
                    goal_pose.pose.position.x=x_map.at(goal_id);
                    goal_pose.pose.position.y=y_map.at(goal_id);
                    x_map.clear();
                    y_map.clear();
                    ROS_INFO("The next laser_goal: X:%f Y:%f",goal_pose.pose.position.x,goal_pose.pose.position.y);
               }
    


            goal_is_frontier_ = false;
            possibly_stuck_ = false;
            //start_laser_nav = false;
            number=0;
        }
        

        // pass goal to move_base
        feedback_.current_target = goal_pose;
        explore_server_.publishFeedback(feedback_);

        move_client_goal_.target_pose = goal_pose;
        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        goal_time_ = ros::Time::now();
        if (explore_server_.isActive())
        {
            move_client_.sendGoal(move_client_goal_,
                                  boost::bind(&ExplorationPlannerROS::doneMovingCb, this, _1, _2),
                                  0,
                                  boost::bind(&ExplorationPlannerROS::feedbackMovingCb, this, _1));
            moving_ = true;
            start_laser_nav=true;
        }
        lock.unlock();

        //wait for movement to finish before continuing
        while(ros::ok() && explore_server_.isActive() && moving_)
        {
            ros::WallDuration(0.1).sleep();
        }

        //laser_nav
        // get robot's current pose 获取机器人位置
        //tf::StampedTransform tf_map_to_robot_base_1;
        //try
        //{
        //   tf_listener_.lookupTransform(params_.map_frame_id_, params_.base_frame_id_,
        //                             ros::Time(0), tf_map_to_robot_base_1);
        //}
        //catch (tf::TransformException ex)
        //{
        //    ROS_ERROR("TF Error: %s",ex.what());
        //    ROS_ERROR("Exploration failed");
        //    explore_server_.setAborted();
        //    return;
        //}
        //PlanarPose robot_planar_pose_1(tf_map_to_robot_base_1.getOrigin().getX(),
        //                             tf_map_to_robot_base_1.getOrigin().getY(),
        //                             tf::getYaw(tf_map_to_robot_base_1.getRotation()) );
        //ROS_INFO("robot_planar_pose_1:X:%f  Y:%f",robot_planar_pose_1.getX(),robot_planar_pose_1.getY());
        //if(fabs
        //      (robot_planar_pose.getX()-robot_planar_pose_1.getX())<1
        //    &&
        //   fabs
        //     (robot_planar_pose.getY()-robot_planar_pose_1.getY())<1
        //  )
        //  {
        //      start_laser_nav=false;
         // number=0;
         // ROS_INFO("number=%d",number);
        //  }
        //if(number==0)
        //{
        //  start_laser_nav=true;
        //  ROS_INFO("start laser navigation");
        //}
    }

    ROS_ASSERT(!explore_server_.isActive());
}

void ExplorationPlannerROS::publishPath(const std::vector<PlanarPose>& p)  //发布路径pm，包含p的开始到结束，x,y，theta
{
    nav_msgs::Path pm;
    pm.header.frame_id = params_.map_frame_id_;
    unsigned int id = 0;
    for (std::vector<PlanarPose>::const_iterator it = p.begin(), iend = p.end(); it != iend; ++it)
    {
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = pm.header.frame_id;
        ps.header.stamp = ros::Time::now();
        ps.header.seq = id++;
        ps.pose.position.x = it->getX();
        ps.pose.position.y = it->getY();
        ps.pose.orientation.w = 1.0;
        pm.poses.push_back(ps);
    }

    path_pub_.publish(pm);
}

void ExplorationPlannerROS::preemptCb()//取消任务
{
    boost::unique_lock<boost::mutex> lock(move_client_lock_);
    move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
    ROS_WARN("Current exploration task cancelled");

    if(explore_server_.isActive()){
        explore_server_.setPreempted();
    }
}

void ExplorationPlannerROS::doneMovingCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result){
//输出当前主动运动结果
    if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
        ROS_ERROR("Failed to move");
        explore_server_.setAborted();
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        moving_ = false;
    }
}

void ExplorationPlannerROS::feedbackMovingCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)//如果不是边界目标任务,执行超时，则取消任务，如果是边界目标，距离小于阈值，也取消任务
{
    ROS_DEBUG("move_base feedback received");
    bool cancel_goal = false;

    // if this is "regular goal" or frontier goal, check if we already executed it for given execution time
    double time_spent_on_goal = (ros::Time::now() - goal_time_).toSec();
    if ( !goal_is_frontier_ && time_spent_on_goal > params_.goal_execution_time)
    {
        ROS_INFO("Current exploration task ran for execution time of %f seconds, cancel it and replan", params_.goal_execution_time);
        cancel_goal = true;
    }

    // if this goal is a frontier, check if we are "close" to it and can move to regular planning again
    double dx = move_client_goal_.target_pose.pose.position.x - feedback->base_position.pose.position.x;
    double dy = move_client_goal_.target_pose.pose.position.y - feedback->base_position.pose.position.y;
    double distance_to_target = std::sqrt( dx*dx + dy*dy );
    if ( goal_is_frontier_ && distance_to_target < params_.frontier_distance_threshold )
    {
        ROS_INFO("Current frontier closer than %f meters, cancel it and replan", params_.frontier_distance_threshold);
        cancel_goal = true;
    }

    if ( cancel_goal )
    {
        boost::unique_lock<boost::mutex> lock(move_client_lock_);
        move_client_.cancelGoalsAtAndBeforeTime(ros::Time::now());
        moving_ = false;
    }

}

void ExplorationPlannerROS::costmapCb(const nav_msgs::OccupancyGrid& map)//检索每一个栅格的map.data是否>=99,是则设置GridValue为true
{
    boost::mutex::scoped_lock lock(costmap_lock_);//出大括号自动解锁

    if (params_.map_frame_id_ != map.header.frame_id)//params_.map_frame_id_不是map.header.frame_id时
    {
        ROS_ERROR("Incoming costmap on topic %s has frame %s, expected %s. Costmap not updated.",
                  costmap_sub_.getTopic().c_str(),
                  map.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    // sparsify the costmap.
    // update instead of rewrite might be more efficient, but rewrite fast enough for now...
    PlanarPose map_origin(map.info.origin.position.x,
                          map.info.origin.position.y,
                          tf::getYaw(map.info.origin.orientation));
    trajectorycheck_map_ = boost::shared_ptr<PlanarGridBinaryMap>(
                new PlanarGridBinaryMap(map_origin, map.info.resolution));

    for (unsigned int h = 0; h < map.info.height; ++h)
    {
        const unsigned int h_offset = h * map.info.width;
        for (unsigned int w = 0; w < map.info.width; ++w)
        {
            // in ROS costmap_2d, costmap_2d::INSCRIBED obstacle corresponds
            // to occupancy 99. We set our threshold there.
            if ( map.data[h_offset + w] != -1 &&
                 map.data[h_offset + w] >= 99 )
            {
                trajectorycheck_map_->setGridValue( PlanarGridIndex(w,h), true);
            }
        }
    }
}

void ExplorationPlannerROS::costmapUpdateCb(const map_msgs::OccupancyGridUpdate &map_update)//检索更新区域每一个栅格的map.data是否>=99,是则设置GridValue为true
{
    boost::mutex::scoped_lock lock(costmap_lock_);

    if (params_.map_frame_id_ != map_update.header.frame_id)
    {
        ROS_ERROR("Incoming costmap on topic %s has frame %s, expected %s. Costmap not updated.",
                  costmap_sub_.getTopic().c_str(),
                  map_update.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    for (unsigned int h = 0; h < map_update.height; ++h)
    {
        const unsigned int h_offset = h * map_update.width;
        for (unsigned int w = 0; w < map_update.width; ++w)
        {
            // in ROS costmap_2d, costmap_2d::INSCRIBED obstacle corresponds
            // to occupancy 99. We set our threshold there.
            if ( map_update.data[h_offset + w] != -1 &&
                 map_update.data[h_offset + w] >= 99 )
            {
                trajectorycheck_map_->setGridValue( PlanarGridIndex( map_update.x + w, map_update.y + h), true);
            }
        }
    }
}


void ExplorationPlannerROS::globalmapCb(const nav_msgs::OccupancyGrid &map)
{
    boost::mutex::scoped_lock lock(global_map_lock_);

    if (params_.map_frame_id_ != map.header.frame_id)
    {
        ROS_ERROR("Incoming global map on topic %s has frame id: %s, expected frame id: %s. Global map not updated.",
                  global_map_sub_.getTopic().c_str(),
                  map.header.frame_id.c_str(),
                  params_.map_frame_id_.c_str());
        return;
    }

    // sparsify the incoming global map
    PlanarPose map_origin(map.info.origin.position.x,
                          map.info.origin.position.y,
                          tf::getYaw(map.info.origin.orientation));
    global_map_ = PlanarGridOccupancyMap(map_origin, map.info.resolution);
    for (unsigned int h = 0; h < map.info.height; ++h)
    {
        const unsigned int h_offset = h*map.info.width;
        for (unsigned int w = 0; w < map.info.width; ++w)
        {
            if ( map.data[h_offset+w] != -1) // -1 implicitly unknown
                global_map_.setGridValue( PlanarGridIndex(w,h),  map.data[h_offset+w]);
        }
    }
}

} // namespace explorationplanner_ros
