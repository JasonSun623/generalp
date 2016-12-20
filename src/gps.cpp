#include <cmath>
#include <vector>

#include <opencv/cv.h>
#include <geodesy/utm.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoint.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <omp.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/Odometry.h>

ros::Publisher p1;
float x,y;
Eigen::Affine3d pose,poseL,poseG;
bool first_run=true;
tf::Transform odomL,odomE,odomE_prev, poseTF,odomG;
ros::Publisher pubPoseOdometry;
tf::TransformBroadcaster *tfBroadcaster;
void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr& navSF_msg)
{
//	std::cout<<pointUTM.easting<<" "<<pointUTM.northing<<std::endl;
	
	geographic_msgs::GeoPoint point = geodesy::toMsg(*navSF_msg);
	geodesy::UTMPoint pointUTM(point);
	poseG.setIdentity();
	tf::poseEigenToTF(poseG,odomG);
	odomG.setOrigin(tf::Vector3(pointUTM.northing,pointUTM.easting,pointUTM.altitude));
		tfBroadcaster->sendTransform(tf::StampedTransform(odomG, ros::Time::now(),"camera_init","gps"));;

		nav_msgs::Odometry gpsOdo;
		gpsOdo.header.frame_id = "/camera_init";
		gpsOdo.child_frame_id = "/gps";
		tf::poseTFToMsg(poseTF, gpsOdo.pose.pose);
		for(int i=0;i<36;i++)
		{
			gpsOdo.pose.covariance[i]=i%6<3&&i<15?navSF_msg->position_covariance[3*floor(i/6)+i%3]:(i%7==0?999:0);
			gpsOdo.twist.covariance[i]=i%7==0?999:0;

		}
		gpsOdo.pose.pose.position.x=pointUTM.northing;
		gpsOdo.pose.pose.position.y=pointUTM.easting;
		gpsOdo.pose.pose.position.z=pointUTM.altitude;
		gpsOdo.header.stamp=ros::Time::now();
		pubPoseOdometry.publish(gpsOdo);
//	std::cout<<pointUTM<<std::endl;
}
void odomLCallBack(const nav_msgs::Odometry::ConstPtr& od)
{
	tf::poseMsgToTF(od->pose.pose,odomL);
	tf::poseTFToEigen(odomL,poseL);
	odomE_prev=odomE;

	Eigen::Affine3d poseDifference;
	tf::poseTFToEigen(poseTF.inverseTimes(odomL),poseDifference);
//	std::cout<<poseDifference.matrix()<<"\n***"<<std::endl;
}
void odomECallBack(const nav_msgs::Odometry::ConstPtr& od)
{
	tf::poseMsgToTF(od->pose.pose,odomE);
	tf::poseTFToEigen(odomE_prev.inverseTimes(odomE),pose);
	pose=pose*poseL;
	//tf::poseEigenToTF(pose,poseTF);
	poseTF=odomL*odomE_prev.inverseTimes(odomE);

		nav_msgs::Odometry laserOdometry;
		laserOdometry.header.frame_id = "/camera_init";
		laserOdometry.child_frame_id = "/camera2";
		tf::poseTFToMsg(poseTF,laserOdometry.pose.pose);
		laserOdometry.header.stamp=ros::Time(0);
		pubPoseOdometry.publish(laserOdometry);
		tfBroadcaster->sendTransform(tf::StampedTransform(poseTF, ros::Time::now(),"camera_init","camera2"));;
}
int main(int argc, char** argv)
{

  	ros::init(argc, argv, "gps");
  	ros::NodeHandle nh("~");
	tf::TransformListener tf_listener;
	ros::Subscriber subGeo = nh.subscribe<sensor_msgs::NavSatFix> ("/gps/fix", 1,gpsHandler);
/*	ros::Subscriber subLOdom = nh.subscribe("/laser_odom_to_init",1,odomLCallBack);
	ros::Subscriber subEOdom = nh.subscribe("/husky_velocity_controller/odom",1,odomECallBack);
*/
	pubPoseOdometry = nh.advertise<nav_msgs::Odometry> ("/gps_odom", 5);
	tfBroadcaster=new (tf::TransformBroadcaster);
	ros::spin();
	/*
	while(true)
	{
		ros::Rate(10).sleep();
		if(first_run)
			{
				tf::StampedTransform tf_now;
				try{
//					tf_listener.waitForTransform("odom","base_link",ros::Time(0),ros::Duration(2.0));
					tf_listener.lookupTransform("/odom","/base_link",ros::Time(0),tf_now);
					tf::poseTFToEigen(tf_now,pose);
				//	first_run=false;
				}
				catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
				}
				std::cout<<"**********"<<std::endl<<pose.translation()<<std::endl;
			}
		ros::spinOnce;
	}
	*/
	return 0;
}

