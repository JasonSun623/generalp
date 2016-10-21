#include <cmath>

#include <generalp/common.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include <pcl/filters/approximate_voxel_grid.h>
#include <ndt_registration/ndt_matcher_p2d.h>
#include <ndt_map/ndt_map.h>
#include <ndt_map/lazy_grid.h>
#include <ndt_map/ndt_cell.h>
#include <ndt_registration/ndt_matcher_d2dl.h>
#include <tf_conversions/tf_eigen.h>
#include <csignal>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>



void run();

int NumInputs=2;

Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
float resolution[2];
double timeLaserCloudFullRes = 0;

float voxel_size[2];
int newLaserCloudFullRes =0;

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > input_cloud;

char rotatingFun[4]={'|','/','-','\\'};
int rotateCount=0;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud, int num)
{
	pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>);

	timeLaserCloudFullRes = laserCloud->header.stamp.toSec();
	pcl::fromROSMsg(*laserCloud, *laserCloudTemp);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*laserCloudTemp,*laserCloudTemp, indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr non_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(*laserCloudTemp,*non_filtered_cloud);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (voxel_size[num], voxel_size[num], voxel_size[num]);
	approximate_voxel_filter.setInputCloud (non_filtered_cloud);
	approximate_voxel_filter.filter (*input_cloud[num]);

	return non_filtered_cloud;
}

void laserCloudFullResHandler0(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
{
	laserHandler(laserCloud,0);
	//std::cout << '\b'<<rotatingFun[++rotateCount%4] << std::flush;
	newLaserCloudFullRes|=1;
}
void laserCloudFullResHandler1(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
{
	laserHandler(laserCloud,1);
	std::cout << '\b'<<rotatingFun[++rotateCount%4] << std::flush;
	newLaserCloudFullRes|=2;
}
#define velodyne

int main(int argc, char** argv)
{
	float skipB,size_x,size_y,size_z;
	ros::init(argc, argv, "laserOdometry");
	ros::NodeHandle nh("~");
		float sensor_range;
		bool doMultires;
		bool do3D=false;
		int numIter;
	nh.param("voxel_size0",voxel_size[0],0.001f);
	nh.param("voxel_size1",voxel_size[1],0.001f);
	nh.param("size_x",size_x,80.05f);
	nh.param("size_y",size_y,80.05f);
	nh.param("size_z",size_z,15.05f);
	nh.param("sensor_range",sensor_range,100.3f);
	nh.param("resolution0", resolution[0],0.40f);
	nh.param("resolution1", resolution[1],0.40f);
	nh.param("numIter", numIter,150);
	std::cout<<std::endl<<std::endl<<"Running NDT  ";

	ros::Subscriber subLaserCloudFullRes0 = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_input0", 2, laserCloudFullResHandler0);
	ros::Subscriber subLaserCloudFullRes1 = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_input1", 2, laserCloudFullResHandler1);
	std::vector<ros::Publisher> pubLaser;
	for(unsigned int i=0;i< NumInputs;i++)
		pubLaser.push_back(nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_"+i, 2));

	ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
	nav_msgs::Odometry laserOdometry;
	laserOdometry.header.frame_id = "/camera_init";
	laserOdometry.child_frame_id = "/camera";

	tf::TransformBroadcaster tfBroadcaster;
	tf::TransformBroadcaster tfBroadcaster2;


	ros::Rate rate(10);
		bool systemInited = false;
	    double pose_init_x=0,pose_init_y=0,pose_init_z=0,
		  pose_init_r=0,pose_init_p=0,pose_init_t=0;
		nh.param("pose_init_r", pose_init_r,pose_init_r);
		nh.param("pose_init_p", pose_init_p,pose_init_p);
		nh.param("pose_init_t", pose_init_t,pose_init_t);
		Eigen::Affine3d pose_, sensor_pose_, Tinit;
		pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		  Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		  Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		  Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
		Tinit.setIdentity();
		ROS_INFO("Init pose is (%lf,%lf,%lf)", pose_.translation()(0), pose_.translation()(1), 
                 pose_.rotation().eulerAngles(0,1,2)(0));

		lslgeneric::NDTMatcherD2DL matcher;

	    matcher.ITR_MAX =numIter;
	    matcher.step_control=true;
		std::vector<lslgeneric::NDTMap > mapLowRes ( NumInputs, new lslgeneric::LazyGrid(resolution[0]));
		std::vector<lslgeneric::NDTMap > mapHighRes ( NumInputs, new lslgeneric::LazyGrid(resolution[1]));
		std::vector<lslgeneric::NDTMap > map ;
		std::vector<lslgeneric::NDTMap > ndlocal ;
			map.push_back(mapLowRes[0]);
			map.push_back(mapHighRes[0]);
			ndlocal.push_back(mapLowRes[1]);
			ndlocal.push_back(mapHighRes[1]);

		for(unsigned int i=0;i< NumInputs;i++)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>) ;
			input_cloud.push_back(tmp);
			map[i].guessSize(0,0,0,size_x,size_y,size_z);
			ndlocal[i].guessSize(0,0,0,size_x,size_y,size_z);
		}	
	    //lslgeneric::NDTMatcherD2D_2D matcher2D;
		Eigen::Affine3d to_cor=pose_.inverse();
	
		ros::Rate(2000).sleep();
	
		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();
			if (__builtin_popcount(newLaserCloudFullRes)!=NumInputs ) 
				continue;
			newLaserCloudFullRes =0;

			if(systemInited)
			{
				for(unsigned int i=0;i<map.size();i++)
				{
					ndlocal[i].loadPointCloud(*input_cloud[i],sensor_range);
					ndlocal[i].computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
				}
				matcher.match( map, ndlocal,Tinit,true) ;
			}
			systemInited=true;
			pose_=pose_*Tinit;
			tf::Transform transform;
			tf::transformEigenToTF(to_cor*pose_, transform);
			tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"camera_init","camera"));;

			tf::Transform transform2;
			tf::transformEigenToTF(pose_, transform2);
			tfBroadcaster2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(),"camera_init","camera2"));;

			for(unsigned int i=0;i<map.size();i++)
			{
				map[i].loadPointCloud(*input_cloud[i],sensor_range);
				map[i].computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

				sensor_msgs::PointCloud2 interestPointsLast2;
				pcl::toROSMsg(*input_cloud[i], interestPointsLast2);
				interestPointsLast2.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
				interestPointsLast2.header.frame_id = "/camera2";
				pubLaser[i].publish(interestPointsLast2);
			}

			pubLaserOdometry.publish(laserOdometry);

		}
}

