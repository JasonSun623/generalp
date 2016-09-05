#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <csignal>
using namespace std;
bool received=false;
std::ofstream ofs;
void synchCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ROS_INFO_STREAM("Synch received");
	received=1;
}
bool term=false;
void int_handler(int x)
{
	term=1;
}

int main(int argc, char** argv)
{
		
    ros::init(argc, argv, "recordT");
    ros::NodeHandle nh("~");
	ros::Publisher synch=nh.advertise<std_msgs::Bool>("/kitti_player/synch",1,true);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> ("/laser_odom_to_init", 1,synchCallback);
//	ros::Subscriber subLaserOdometry = nh.subscribe ("/aft_mapped_to_init", 2,synchCallback);
	std_msgs::Bool reply;
	reply.data=1;
	tf::StampedTransform transform;
	tf::TransformListener listener;
	time_t timer[2];
	int i=0;
	time(&timer[0]); time(&timer[1]);
	signal(SIGINT,int_handler);
	ofs.open(argv[1],std::ofstream::out);
	ofs<<std::scientific;
	ros::Duration(5).sleep();
	while(1){
		ros::spinOnce();
		if(term)break;
		 
		//if(!received&&(!time(&timer[i%2])||difftime(timer[i%2],timer[(i+1)%2])<5)) 
		if(!received)
		{
			ros::Duration(0.1).sleep();
			continue;
		}
		received=0;
		synch.publish(reply);
		i++;
		try
		{
			listener.lookupTransform("camera_init","/camera",ros::Time(0),transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			continue;
		}
		Eigen::Matrix4f m;
		pcl_ros::transformAsMatrix(transform, m);
		for(int k=0;k<3;k++)
			for(int j=0;j<4;j++)
				ofs<< double (m(k,j))<<" ";
		ofs<<endl;
		for(int k=0;k<4;k++)
			for(int j=0;j<4;j++)
				std::cout<< double (m(k,j))<<" ";
		std::cout<<endl;
	}
	ofs.close();
	ros::shutdown();
	return 0;

}
