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
#include <ndt_registration/ndt_matcher_d2d.h>
#include <tf_conversions/tf_eigen.h>
#include <csignal>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ndt_registration/ndt_matcher_d2d_2d.h>



void run();


Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
float resolution;
double timeLaserCloudFullRes = 0;

float voxel_size;
bool newLaserCloudFullRes = false;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());

char rotatingFun[4]={'|','/','-','\\'};
int rotateCount=0;


void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
	pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>);

	timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
	pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudTemp);
	  std::vector<int> indices;
	  pcl::removeNaNFromPointCloud(*laserCloudTemp,*laserCloudTemp, indices);
	pcl::PointCloud<pcl::PointXYZ>::Ptr non_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(*laserCloudTemp,*non_filtered_cloud);
	pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
	approximate_voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
	approximate_voxel_filter.setInputCloud (non_filtered_cloud);
	approximate_voxel_filter.filter (*input_cloud);
	std::cout << '\b'<<rotatingFun[++rotateCount%4] << std::flush;

  newLaserCloudFullRes = true;
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
	nh.param("voxel_size",voxel_size,0.05f);
	nh.param("size_x",size_x,80.05f);
	nh.param("size_y",size_y,80.05f);
	nh.param("size_z",size_z,15.05f);
	nh.param("sensor_range",sensor_range,100.3f);
	nh.param("resolution", resolution,0.40f);
	nh.param("numIter", numIter,150);
	nh.param("doMultires", doMultires,true);
	nh.param("do3D", do3D,false);
	std::cout<<std::endl<<std::endl<<"Running NDT  ";

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 2, laserCloudFullResHandler);
	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_3", 2);


	ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
	nav_msgs::Odometry laserOdometry;
	laserOdometry.header.frame_id = "/camera_init";
	laserOdometry.child_frame_id = "/camera";

	tf::TransformBroadcaster tfBroadcaster;
	tf::TransformBroadcaster tfBroadcaster2;


	ros::Rate rate(100);
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

		lslgeneric::NDTMatcherD2D matcher;

	    matcher.ITR_MAX =numIter;
	    matcher.step_control=true;
	    lslgeneric::NDTMap map(new lslgeneric::LazyGrid(resolution));
	    map.guessSize(0,0,0,sensor_range,sensor_range,size_z);
	    lslgeneric::NDTMatcherD2D_2D matcher2D;
		Eigen::Affine3d to_cor=pose_.inverse();
	
	
		while(ros::ok())
		{
			rate.sleep();
			ros::spinOnce();
			if (!newLaserCloudFullRes ) 
				continue;
			newLaserCloudFullRes = false;

			if (!systemInited) 
			{
				map.loadPointCloud(*input_cloud,sensor_range);
				map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

				sensor_msgs::PointCloud2 interestPointsLast2;
				pcl::toROSMsg(*input_cloud, interestPointsLast2);
				interestPointsLast2.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
				interestPointsLast2.header.frame_id = "/camera";
				pubLaserCloudFullRes.publish(interestPointsLast2);

				systemInited = true;
				continue;
			}
			

			if(doMultires){
				lslgeneric::NDTMap ndlocalLow(new lslgeneric::LazyGrid(3*resolution));
				ndlocalLow.guessSize(0,0,0,sensor_range,sensor_range,size_z);
				ndlocalLow.loadPointCloud(*input_cloud,sensor_range);
				ndlocalLow.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);


				lslgeneric::NDTMap mapLow(new lslgeneric::LazyGrid(3*resolution));
				mapLow.initialize(0,0,0,3*size_x,3*size_y,size_z);
				std::vector<lslgeneric::NDTCell*> ndts;
				ndts = map.getAllCells(); //this copies cells?
				for(int i=0; i<ndts.size(); i++)	
				{
					lslgeneric::NDTCell *cell = ndts[i];
					if(cell!=NULL)
					{
						if(cell->hasGaussian_)
						{
						Eigen::Vector3d m = cell->getMean();	
						Eigen::Matrix3d cov = cell->getCov();
						unsigned int nump = cell->getN();
						mapLow.addDistributionToCell(cov, m,nump);
						}
					}
					delete cell;
				}
				matcher2D.match( mapLow, ndlocalLow,Tinit,true);
			}

			if(do3D)
			{
				lslgeneric::NDTMap ndlocal(new lslgeneric::LazyGrid(resolution));
				ndlocal.guessSize(0,0,0,sensor_range,sensor_range,size_z);
				ndlocal.loadPointCloud(*input_cloud,sensor_range);
				ndlocal.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
				matcher.match( map, ndlocal,Tinit,true) ;
			}
			pose_=pose_*Tinit;

			map.loadPointCloud(*input_cloud,sensor_range);
			map.computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);

			tf::Transform transform;
			tf::transformEigenToTF(to_cor*pose_, transform);
			tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"camera_init","camera"));;

			tf::Transform transform2;
			tf::transformEigenToTF(pose_, transform2);
			tfBroadcaster2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(),"camera_init","camera2"));;

			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*input_cloud, laserCloudFullRes3);
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
			laserCloudFullRes3.header.frame_id = "/camera2";
			pubLaserCloudFullRes.publish(laserCloudFullRes3);

			pubLaserOdometry.publish(laserOdometry);

		}
}

