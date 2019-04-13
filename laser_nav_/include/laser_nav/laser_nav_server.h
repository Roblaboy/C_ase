
#define LASER_NAV_SERVER_H
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <laser_nav_/laser_nav_srv.h>

#include<math.h>
    class laser_nav
    {
        public:
        void transformPoint();
        double ori_cal(int a,int b);
        bool laser_nav(laser_nav_::laser_nav_srv::Request &req,
               laser_nav_::laser_nav_srv::Response &res);
        ros::ServiceServer service;
        ros::NodeHandle n;
        tf::TransformListener listener;

        geometry_msgs::PointStamped laser_goal_ONbaseframe;
        geometry_msgs::PointStamped laser_goal_ONmapframe;
        ros::Publisher goal_pose_pub_;
        

        
    };

