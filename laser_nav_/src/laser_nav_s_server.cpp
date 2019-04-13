#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include <stdio.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <laser_nav_/laser_nav_srv.h>
#include <math.h>
#include <visualization_msgs/Marker.h>

using namespace std;
//geometry_msgs::PointStamped laser_goal_ONbaseframe;
//geometry_msgs::PointStamped laser_goal_ONmapframe;
//ros::Publisher goal_pose_pub_;

//void transformPoint();
//double ori_cal(int a,int b);
float Pi=3.141592654;
float max_dist=5.5;
float Robot_max_dia=0.6;
float omiga=Pi/6;
int   laser_length=650;
float laser_angle_step_deg=0.365;

class laser_nav
{
    public:    
     laser_nav();
    private:
    
    ros::NodeHandle n;
    tf::TransformListener listener;
    ros::ServiceServer service;
    ros::Subscriber scan_sub;
    ros::Publisher laser_goal_pub;

    bool laser_nav_server(laser_nav_::laser_nav_srv::Request &req,
            laser_nav_::laser_nav_srv::Response &res);
        
    
    //visualization_msgs::Marker marker;
    float ori_cal(int a,int b);
   
    int seq_point_numbers();//对应最远处允许的第一类边界的最小长度的激光点数
    std::vector<float> x,y;
    float *laser_data;  
    geometry_msgs::PointStamped goal_pose_base;
    geometry_msgs::PointStamped goal_pose_map;
    void scancallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool ifpassable(int a,int m,float h,bool ori);//检测第二类目标点是否可达
};

int laser_nav::seq_point_numbers()
{
    int num=0;
    //num=Robot_max_dia/max_dist/Pi*180/laser_angle_step_deg+1;
    num = 180/Pi*acos(double(1-pow(2*Robot_max_dia,2)/(2*max_dist*max_dist)))/laser_angle_step_deg;
    ROS_INFO("num=%d",num);
    return num;
}

bool laser_nav::ifpassable(int a,int m,float h,bool ori)
{
    ROS_INFO("m = %d",m);
    bool ifpassable = false;
    int count = 0;
    if(ori)//ori为true为逆时针,false，则方向为顺时针转
    {
        for(int i=a;i<a+m;i++)
        {
            if(laser_data[i] >= h)
            count++;
        }
    }
    else
    {
        for(int i=a;i>a-m;i--)
        {
            if(laser_data[i] >= h)
            count++;
        }
    }
    if(count == m)
    {
        ifpassable = true;
        ROS_INFO("pass");
    }
    return ifpassable;
}

bool laser_nav::laser_nav_server(laser_nav_::laser_nav_srv::Request &req,
               laser_nav_::laser_nav_srv::Response &res)
{    
    //设置Marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = req.map_frame_id;
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
    //**************************************************//
    
    //******************逆时针分析激光数据*****************//
    int ranges,disc=0;
    int start_n,end_n,j=0;
    float theta=0.0;
    std::vector<float> x,y;
    ranges=laser_length;
    for(int i=1;i<ranges;i++)
    {
    //****************一类目标点*********************//
        if(laser_data[i]>=max_dist) 
        {
            start_n=i;//标记弧线起始激光位置
            for(j=i;j<ranges;++j)
            {
                if(laser_data[j]>=max_dist&&j!=(ranges-1))
                {
                    ++disc;
                }
                else if(disc > seq_point_numbers()) 
                {
                    end_n=j;//遇到第一个不连续点，并且在这之前已经有连续的14个最远点，标记此弧线末端激光位置
                    ROS_INFO("i:%d j:%d",i,j);
                    theta=ori_cal(i,j);
                    x.push_back(5.0*sin(theta-omiga));
                    y.push_back(-5.0*cos(theta-omiga));                   
                    disc=0;
                    i=j;
                    break;//扫完一段边界，跳出这层for循环
                }
                else break;//遇到第一个不连续点，并且在这之前没有连续的n个最远点，跳出这层for循环，继续扫描下一段
            }
        }
        //****************二类目标点*********************//
        if(fabs(laser_data[i]-laser_data[i-1])>=1.5*Robot_max_dia)
        {
            if((laser_data[i]<max_dist)&&(laser_data[i-1]<max_dist))
            {
                if(laser_data[i-1] > laser_data[i])
                {
                    /**********改动部分**********/
                    float max_dist = laser_data[i-1];
                    float min_dist = laser_data[i];
                    int max_id = i-1;
                    int min_id = i;
                    int m = 180/Pi*2*asin(1/2*Robot_max_dia/(2*min_dist))/laser_angle_step_deg;//计算文章中公式11中的m
                    theta = laser_nav::ori_cal(max_id,max_id)-acos(1-0.5*Robot_max_dia/2/pow(max_dist+min_dist,2));
                    float X = 0.5*(max_dist+min_dist)*sin(theta-omiga);
                    float Y = -0.5*(max_dist+min_dist)*cos(theta-omiga);
                    float h = sqrt(X*X+Y*Y)+0.5*Robot_max_dia;
                    if(ifpassable(max_id,m,h,false))
                    {
                        x.push_back(X);
                        y.push_back(Y);
                    }
                }
                else
                {
                    float max_dist = laser_data[i];
                    float min_dist = laser_data[i-1];
                    int max_id = i;
                    int min_id = i-1;
                    int m = 180/Pi*2*asin(1/2*Robot_max_dia/(2*min_dist))/laser_angle_step_deg;
                    theta = laser_nav::ori_cal(max_id,max_id)+acos(1-0.5*Robot_max_dia/2/pow(max_dist+min_dist,2));
                    float X = 0.5*(max_dist+min_dist)*sin(theta-omiga);
                    float Y = -0.5*(max_dist+min_dist)*cos(theta-omiga);
                    float h = sqrt(X*X+Y*Y)+0.5*Robot_max_dia;
                    if(ifpassable(max_id,m,h,true))
                    {
                        x.push_back(X);
                        y.push_back(Y);
                    }
                    /***************************/
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
            goal_pose_base.header.frame_id=req.base_frame_id;
            goal_pose_base.point.x=x.at(i);
            goal_pose_base.point.y=y.at(i);
            goal_pose_base.point.z=0;
            goal_pose_base.header.stamp=ros::Time(0);
            try
            {
                listener.transformPoint(req.map_frame_id, goal_pose_base, goal_pose_map);
            }
            catch (tf::TransformException &ex) 
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            res.x_map.push_back(goal_pose_map.point.x);
            res.y_map.push_back(goal_pose_map.point.y);
        }     
        for(int i=0;i<res.x_map.size();++i)  
        {
            geometry_msgs::Point p;
            p.x=res.x_map.at(i);
            p.y=res.y_map.at(i);
            p.z=0.0;
            marker.points.push_back(p);    
        }
        laser_goal_pub.publish(marker);   
        x.clear();
        y.clear();
        return true;
    }
    else
    {
        laser_goal_pub.publish(marker);
        return false;
    }
    
}



laser_nav::laser_nav()
{
    //n.param<float>("laser_angle_step_deg",laser_angle_step_deg,0.36);
    //n.param<int>("laser_length",laser_length,666);
    laser_data = new float[laser_length];
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1,&laser_nav::scancallback,this);
    service = n.advertiseService("select_laser_nav_goal", &laser_nav::laser_nav_server,this);
    laser_goal_pub = n.advertise<visualization_msgs::Marker>( "laser_goal", 10 );
}

void laser_nav::scancallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ros::Rate r(10);
    int ranges=laser_length;
    //ROS_INFO("length=%d",ranges);
    for(int i=0;i<ranges;i++)
    {
        laser_data[i]=scan->ranges.at(i);
    }
    r.sleep();
    //laser_data.clear();
}

float laser_nav::ori_cal(int a,int b)
{
    float rad=0;
    float center_id=0.0;
    center_id=(b+a)/2;
    rad=laser_angle_step_deg*center_id*Pi/180;
    return rad;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "select_laser_nav_goal_server");
    laser_nav laser_nav;
    ROS_INFO("Ready to select a goal from laser data.");
    ros::spin();
  
}
