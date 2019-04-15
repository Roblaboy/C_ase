# C_ase
Autonomous Exploration
This software contained in the active worksapce. This workspace contain three packages including c_ase_exploration, laser_nav_, 
topology. It is a ROS integrated planner for robotic exploration tasks, taking as input data from laser data and map data and outputting
new exploration targets for the robot. To excute the exploration task, this software uses move_base. In order to use this software, you 
will need to have properly configured move_base to run on your robot.

Simulator demo: Turtlebot exploring an environment
You can test the exploration planner by running it using simulator. Here, we apply mostly standard settings from the turtlebot_stage package. To run the demo, you need to install the dependencies:

sudo apt-get install ros-indigo-turtlebot-simulator ros-indigo-frontier-exploration ros-indigo-turtlebot-navigation ros-indigo-turtlebot-stage
（注意，indigo为安装的ROS版本，请自行替换为对应的版本名）
Before continuing, make sure you have sourced setup.bash of the catkin workspace you installed the exploration package in, do this as follows (changing the path appropriately):

source ~/{your workspace}/devel/setup.bash

Now run the main launch file:

roslaunch ase_exploration top_simulator_exploration.launch

The simulator will now start, along with other nodes and an rviz visualization.

Finally, start the action client to send an exploration command to the robot. This command must be run in a terminal window where setup.bash of the catkin workspace you installed the exploration package in has been sourced.

rosrun actionlib axclient.py /exploration
Click on "Send", and the simulated Turtlebot will start exploring the environment.

ROS Node details: ase_exploration_planner_node
1.Actions provided
This node provides an implementation of the SimpleActionServer for the ExploreAction type defined in this package. This action is provided with the name exploration. Your best bet to get started is probably to use actionlib's axclient to send an action request to start exploration: rosrun actionlib axclient.py /exploration.

2.Actions called
The exploration targets produced by the planner are sent as navigation goals to move_base. This is handled by a SimpleActionClient calling an move_base_msgs::MoveBaseAction on the topic move_base.

3. Services required
This node requires and calls one service from the topology package. It is:
topology/top
and then, this service will call the other service from the laser_nav_ package. It is:
laser_nav_/laser_nav_srv
