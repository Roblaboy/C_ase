#include<math.h> 
#include<algorithm>
#include <topology/topology.h>
#include "std_msgs/String.h"
#include <sstream>
#include <frontier_exploration/frontier_search.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <frontier_exploration/costmap_tools.h>
#include <geometry_msgs/Point.h>
#include <boost/foreach.hpp>
#include <visualization_msgs/Marker.h>
#include <topology/topology_msg.h>
#include <frontier_exploration/costmap_tools.h>
#include <nav_msgs/Odometry.h>
// Map subscribing
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <topology/top.h>
#include <topology/laser_nav_srv.h>
//query
#include <tree_traverse/tree_traverse.h>
namespace topology{

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;	

class Topo
{
public:
    Topo();
	
	
private:
    ros::NodeHandle nt;
    ros::Subscriber map_sub;
    ros::Subscriber goal_sub;
	ros::Subscriber final_goal_sub;
	ros::Subscriber current_pose_sub;

	ros::Publisher  top_pub;
	ros::Publisher  top_c_pub;
	ros::Publisher  top_goal_pub;

	ros::ServiceServer service;

	topology_msg    top_map;
	
	tf::TransformListener listener;
	geometry_msgs::PointStamped current_point_odom;
	geometry_msgs::PointStamped current_point_map;
	geometry_msgs::PoseStamped  Next_goal;

	boost::mutex current_point_odom_lock;
	boost::mutex map_lock;
	nav_msgs::OccupancyGrid map;
	int id_;
	bool set_top;
	bool initial_goal;
	int Threshold_value;
	double pi;
	double r;

	//拓扑图起始点,也是tree矩阵记录的第一个点
	int                                current;
	//下面三个map数据分别记录了每一个id对应的坐标值,父子节点关系，增益值
	std::map<int,geometry_msgs::Point>	tmap_point;
	std::map<int,int>	                tmap_order;
	std::map<int,int>	                tmap_value;

	void get_map(const nav_msgs::OccupancyGrid &map_);
    bool set_topology(top::Request &req,top::Response &res);
	void top_tree_update(const nav_msgs::OccupancyGrid &map_);
	void current_pose_sub_callback(const nav_msgs::Odometry &C_pose);
	void Marker_initialize(int id,unsigned char color,unsigned char type,visualization_msgs::Marker &marker);
	void marker_top_pub(visualization_msgs::Marker marker);
	void marker_top_c_pub(visualization_msgs::Marker marker);
	int Traverse(int index,const nav_msgs::OccupancyGrid &map_);
	int Max_value_index();
	int find_position(std::vector<geometry_msgs::Point> v,geometry_msgs::Point p);
	std::vector<unsigned int> map_nhood4(unsigned int idx,const nav_msgs::OccupancyGrid &map);
};
Topo::Topo():
listener(ros::Duration(10.0)),
current(0)
{
	id_=0;
	set_top=false;
	pi=3.141592654;
	r=1.5;
	Threshold_value=999;
	initial_goal = true;
	map_sub = nt.subscribe("map", 1, &Topo::get_map,this);
	current_pose_sub = nt.subscribe("odom",1,&Topo::current_pose_sub_callback,this);
	top_pub = nt.advertise<visualization_msgs::Marker>("top_map",200); 
	top_c_pub = nt.advertise<visualization_msgs::Marker>("top_c_map",200);  
	top_goal_pub = nt.advertise<geometry_msgs::PoseStamped>("top_goal_pose", 1);
	service = nt.advertiseService("set_top_point",&Topo::set_topology,this);
}

void Topo::top_tree_update(const nav_msgs::OccupancyGrid &map_)
{
	boost::mutex::scoped_lock lock(current_point_odom_lock);
	try
	{
		listener.transformPoint(map_.header.frame_id, current_point_odom, current_point_map);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
       	ros::Duration(1.0).sleep();
    }
	if (map_.header.frame_id != current_point_map.header.frame_id)
	{
		ROS_ERROR("The header of current_point_map is %s , expected %s. ",
				current_point_map.header.frame_id.c_str(),
				map_.header.frame_id.c_str());
		return;
	}
	
	
	if(!tmap_point.empty())
	{
		std::map<int,int>::iterator it=tmap_value.begin();
<<<<<<< HEAD
		it++;
=======
>>>>>>> 6029deb869c6703609388e283226548292b791b7
		for(it;it!=tmap_value.end();it++)
		{
			if(it->second != 0 && it->second != -1)
			{
				int index = (int)(tmap_point.at(it->first).x-map_.info.origin.position.x)/map_.info.resolution+(int)
			              (tmap_point.at(it->first).y-map_.info.origin.position.y)/map_.info.resolution*map_.info.width;
				tmap_value.at(it->first) = Traverse(index,map_); //修改增益值
				ROS_INFO("top %d gain change:%d",it->first,tmap_value.at(it->first));
				if(tmap_value.at(it->first) <= Threshold_value)
				{
					tmap_value.at(it->first) = -1;
				}
			}
			
		}
	}
	
}

int Topo::Traverse(int index,const nav_msgs::OccupancyGrid &map)
{
	int n=0;
	int value=0;
	unsigned int size_x_ , size_y_;
	double resolution=map.info.resolution;//获取代价地图的分辨率
	int num_max=int(pi*r*r/resolution/resolution);//计算每个候选点的扫描范围内所需遍历的栅格数
	
	size_x_ = map.info.width;
	size_y_ = map.info.height;
	std::vector<bool> visited_flag(size_x_ * size_y_, false);

	//initialize breadth first search
	std::queue<unsigned int> bfs;
	
	//ROS_INFO("idx:%d",idx);
	bfs.push(index);
	while (!bfs.empty()&&n<num_max)
	{
		int idx = bfs.front();
		bfs.pop();
		BOOST_FOREACH(unsigned int nbr, map_nhood4(idx, map))
		{
			//ROS_INFO("%d",map[nbr]);
			if(map.data[nbr]<=0&&!visited_flag[nbr])
			{
				visited_flag[nbr] = true;
				bfs.push(nbr);
				n++;
				if(map.data[nbr]==-1)
				{
					value++;
				}
			}
		}
	}
	return value;
}

std::vector<unsigned int> Topo::map_nhood4(unsigned int idx,const nav_msgs::OccupancyGrid &map)
{
	std::vector<unsigned int> out;

    unsigned int size_x_ = map.info.width, size_y_ = map.info.height;

    if (idx > size_x_ * size_y_ -1)
	{
        ROS_WARN("Evaluating nhood for offmap point");
        return out;
    }

    if(idx % size_x_ > 0)            {out.push_back(idx - 1);}
    if(idx % size_x_ < size_x_ - 1)  {out.push_back(idx + 1);}
    if(idx >= size_x_)               {out.push_back(idx - size_x_);}
    if(idx < size_x_*(size_y_-1))    {out.push_back(idx + size_x_);}
    return out;
}

bool Topo::set_topology(top::Request &req,top::Response &res)
{
	static int MaxGainId = 0;
	static int last_id = 0;
	ROS_INFO("get laser points");
	std::queue<geometry_msgs::Point> laser_goal;
	laser_nav_srv top_srv;
	ros::ServiceClient top_nav = 
                   nt.serviceClient<laser_nav_srv>("select_laser_nav_goal");
	top_srv.request.base_frame_id = "base_link";
    top_srv.request.map_frame_id = map.header.frame_id;
	if(top_nav.call(top_srv))
    {
<<<<<<< HEAD
        ROS_INFO("Got candidate target point collection from laser_navigation");
=======
        ROS_INFO("Got next goal collection from laser_navigation");
>>>>>>> 6029deb869c6703609388e283226548292b791b7
                
        //ROS_WARN("X:%f Y:%f",goal_pose.pose.position.x,goal_pose.pose.position.y);
		geometry_msgs::Point m;
        for(int i=0;i<top_srv.response.x_map.size();++i)
        {
            m.x = top_srv.response.x_map.at(i);
            m.y = top_srv.response.y_map.at(i);
			laser_goal.push(m);
        }
    }
    else
    {
        ROS_ERROR("Call to service %s failed", top_nav.getService().c_str() );
    }
	boost::mutex::scoped_lock lock(map_lock);
		ROS_INFO("\n/************************ top start **************************/");
<<<<<<< HEAD
=======
		set_top = false;
>>>>>>> 6029deb869c6703609388e283226548292b791b7
		visualization_msgs::Marker top_marker;
		visualization_msgs::Marker top_c_marker;
		
		//设置Marker
		Marker_initialize(1,'r','l',top_marker);//红，拓扑图
		Marker_initialize(0,'b','l',top_c_marker);//蓝，候选拓扑图
		/************************/
		double resolution=map.info.resolution; 
		ROS_INFO("num_max:%d",int(pi*r*r/resolution/resolution));
		top_tree_update(map);

		//将起始点加入到第一个拓扑点
		
		if(initial_goal)
		{
			tmap_point[0] = current_point_map.point;
			tmap_value[0] = 0;
			initial_goal = false;
			
		}
		else//不是初始点，则把当前位置设为最新top点
		{
			id_++;
			tmap_point[id_] = current_point_map.point;
			tmap_value[id_] = 0;
			tmap_order[id_] = last_id;//last_id为上一轮选中的候选top节点的父节点
<<<<<<< HEAD
			//top_goal_pub.publish(current_point_map.point);
			/*****   发布top图到RVIZ  ***********/
			marker_top_pub(top_marker);
			/*****   发布candidate top map到RVIZ  ***********/
			marker_top_c_pub(top_c_marker);
		}
					
=======
		}
>>>>>>> 6029deb869c6703609388e283226548292b791b7
		current = id_;
		while(!laser_goal.empty())      //扫描栅格得到每个候选点的增益值
		{   
			//计算各候选目标点map下的index,添加候选top节点
			geometry_msgs::Point p = laser_goal.front();
			laser_goal.pop();
			unsigned int idx=(int)(p.x-map.info.origin.position.x)/resolution+(int)
			              (p.y-map.info.origin.position.y)/resolution*map.info.width;
			
			int value = Traverse(idx,map);
						  
			if(value >= Threshold_value)      //如果该增益值高于候选top阈值，把该点加入到top_map的c_top队列
			{
				id_++;
				tmap_point[id_] = p;
				tmap_value[id_] = value;
				tmap_order[id_] = current;
				ROS_INFO("top_map cost value: %d > %d,push it into top_map's candidate queue",value,Threshold_value);
			}
			else
			{
				ROS_INFO("top_map cost value: %d < %d,abandoned",value,Threshold_value);
			}
<<<<<<< HEAD
			/*****   发布candidate top map到RVIZ  ***********/
			marker_top_c_pub(top_c_marker);
=======
>>>>>>> 6029deb869c6703609388e283226548292b791b7
			
		}
		//计算增益值最大的目标点
		MaxGainId = Max_value_index();//获取当前候选节点中增益值最大的候选目标点
<<<<<<< HEAD
		//检测是否over
		if(MaxGainId!=0)
		{
			ROS_INFO("Next_goal id: %d ",MaxGainId);
			last_id = tmap_order[MaxGainId];//为下一次更新top图做准备

			//计算下个目标点的位姿
			Next_goal.header.frame_id = map.header.frame_id;
			Next_goal.pose.position = tmap_point.at(MaxGainId);
			int father_index = tmap_order.at(MaxGainId);
			ROS_INFO("father_index:%d",father_index);
			geometry_msgs::Point father = tmap_point.at(father_index) ;
			double dx = tmap_point.at(MaxGainId).x - father.x;
			double dy = tmap_point.at(MaxGainId).y - father.y;
			double theta = atan2(dy,dx);
			Next_goal.pose.orientation.x = 0.0;
			Next_goal.pose.orientation.y = 0.0;
			Next_goal.pose.orientation.z = sin(theta/2);
			Next_goal.pose.orientation.w = cos(theta/2);

			/***********发布目标点**************/
			top_goal_pub.publish(Next_goal);
			res.Next_goal = Next_goal;


			
			//将选中的候选目标点放弃，不进行以后的top map update,记录他的父节点，为下次记录新的top点做准备
			tmap_value.at(MaxGainId) = -1;
			return true;
		}
		else
		{
			/*id_++;
			tmap_point[id_] = current_point_odom.point;
			tmap_value[id_] = 0;
			tmap_order[id_] = last_id;

			*****   发布top图到RVIZ  ***********
			marker_top_pub(top_marker);
			marker_top_c_pub(top_c_marker);*/
			return false;
		}
		
=======
		ROS_INFO("Next_goal id: %d ",MaxGainId);
		/*****   发布candidate top map到RVIZ  ***********/
		marker_top_c_pub(top_c_marker);
		//将选中的候选目标点放弃，不进行以后的top map update,记录他的父节点，为下次记录新的top点做准备
		tmap_value.at(MaxGainId) = -1;
		last_id = tmap_order[MaxGainId];
		//current = MaxGainId;//为下一次更新top图做准备,此处做了修改

		//计算下个目标点的位姿
		Next_goal.header.frame_id = map.header.frame_id;
		Next_goal.pose.position = tmap_point.at(MaxGainId);
		int father_index = tmap_order.at(MaxGainId);
		ROS_INFO("father_index:%d",father_index);
		geometry_msgs::Point father = tmap_point.at(father_index) ;
		double dx = tmap_point.at(MaxGainId).x - father.x;
		double dy = tmap_point.at(MaxGainId).y - father.y;
		double theta = atan2(dy,dx);
		Next_goal.pose.orientation.x = 0.0;
		Next_goal.pose.orientation.y = 0.0;
		Next_goal.pose.orientation.z = sin(theta/2);
		Next_goal.pose.orientation.w = cos(theta/2);

		/***********发布目标点**************/
		top_goal_pub.publish(Next_goal);
		res.Next_goal = Next_goal;

		/*****   发布top图到RVIZ  ***********/
		marker_top_pub(top_marker);
		return true;
>>>>>>> 6029deb869c6703609388e283226548292b791b7
	
	
}


void Topo::get_map(const nav_msgs::OccupancyGrid &map_)
{
	ros::Rate r(1);
	boost::mutex::scoped_lock lock(map_lock);
	map.header.frame_id = map_.header.frame_id;
	map.info  = map_.info;
	map.data.resize(map_.data.size());
	for(int i=0;i<map_.data.size();i++)
	{
		map.data.at(i)=map_.data.at(i);
	}
	//&map = &map_;
	r.sleep();
}

void Topo::current_pose_sub_callback(const nav_msgs::Odometry &C_pose)
{
	boost::mutex::scoped_lock lock(current_point_odom_lock);
	//ROS_INFO("get current_point_odom");
	current_point_odom.header = C_pose.header;
	//ROS_INFO("current_point_odom.header.frame_id:%s",current_point_odom.header.frame_id.c_str());
	current_point_odom.point = C_pose.pose.pose.position;
	//ROS_INFO("%lf %lf",current_point_odom.point.x,current_point_odom.point.y);
}

int Topo::Max_value_index()
{
	int pos = 0;
	double gain = 0;
	std::vector<int> line;
	std::map<int,int>::iterator it=tmap_value.begin();
<<<<<<< HEAD
	it++;
=======
>>>>>>> 6029deb869c6703609388e283226548292b791b7
	for(it;it!=tmap_value.end();it++)
	{
		if(it->second != 0)
		{
			int Next = it->first;
			line = query(tmap_order,current,Next);//获取拓扑路径
			double distance = 0.0;
			for(int i = 0; i<line.size()-1;i++)
			{
				int m = line.at(i);
				int n = line.at(i+1);
				distance += sqrt(pow(tmap_point.at(m).x-tmap_point.at(n).x,2)+pow(tmap_point.at(m).y-tmap_point.at(n).y,2));//计算拓扑距离
				//ROS_INFO("The distance from point %d to point %d is %f",m,n,distance);
			}
			if(gain < double(tmap_value.at(Next))*exp(-distance/5))
			{
				gain = double(tmap_value.at(Next))*exp(-distance/5);
				pos = Next;
			}
		}
		
	}
	//ROS_INFO("The max gain is %f",gain);
	return pos;
}

void Topo::Marker_initialize(int id,unsigned char color,unsigned char type,visualization_msgs::Marker &marker)
{
	marker.header.frame_id = map.header.frame_id;
	marker.header.stamp = ros::Time(0);
	marker.ns="topology_sub";
	marker.id=id;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.w=1.0;
	marker.scale.x = 0.2;  
	marker.scale.y = 0.2;
	marker.scale.z = 0;

	switch (type)
	{
		case 'p':
			marker.type = visualization_msgs::Marker::POINTS;
		case 'l':
			marker.type = visualization_msgs::Marker::LINE_LIST;
	}

	switch (color)
	{
		case 'r':	
			marker.color.g=0.0;
			marker.color.a=1.0;
			marker.color.r = 1.0;
			marker.color.b = 0.0;
			break;
		case 'b':
			marker.color.g=0.0;
			marker.color.a=1.0;
			marker.color.r = 0.0;
			marker.color.b = 1.0;
			break;
		case 'g':
			marker.color.g=1.0;
			marker.color.a=1.0;
			marker.color.r = 0.0;
			marker.color.b = 0.0;
			break;
	}	
}

void Topo::marker_top_pub(visualization_msgs::Marker marker)
{
	marker.points.clear();
	std::map<int,int>::iterator it=tmap_value.begin();
	it++;           //从第一个目标点开始，而不是起始点
	for(it;it!=tmap_value.end();it++)
	{
		if(it->second == 0 && tmap_value.at(tmap_order.at(it->first))==0)//该点增益值为0，并且其父节点的增益值也为0
		{
			int son = it->first;
			int fat = tmap_order.at(it->first);
			marker.points.push_back(tmap_point.at(son));
			marker.points.push_back(tmap_point.at(fat));
		}
	}
	top_pub.publish(marker);
}
void Topo::marker_top_c_pub(visualization_msgs::Marker marker)
{
	marker.points.clear();
	std::map<int,int>::iterator it=tmap_value.begin();
	it++;      //从第一个目标点开始，而不是起始点
	for(it;it!=tmap_value.end();it++)
	{
		if(it->second != 0 && it->second != -1)//该点增益值不为0
		{
			int son = it->first;
			int fat = tmap_order.at(it->first);
			marker.points.push_back(tmap_point.at(son));
			marker.points.push_back(tmap_point.at(fat));
		}
	}
	top_pub.publish(marker);
}


}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "topology_sub");
	topology::Topo top;
	ROS_INFO("start");
	ros::spin();

	return 0;
}


