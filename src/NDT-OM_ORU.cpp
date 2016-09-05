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
#include <ndt_fuser/ndt_fuser_hmt.h>
#include <tf_conversions/tf_eigen.h>
#include <csignal>
#include <ndt_map/NDTMapMsg.h>
#include <ndt_map/ndt_conversions.h>
#include <nav_msgs/OccupancyGrid.h>



void run();


Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
float resolution;
int numIter;
double timeLaserCloudFullRes = 0;
double timeImuTrans = 0;

float voxel_size;
bool newLaserCloudFullRes = false;
bool newImuTrans = false;
bool reset=false;

pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>());

char rotatingFun[4]={'|','/','-','\\'};
int rotateCount=0;


float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuShiftFromStartX = 0, imuShiftFromStartY = 0, imuShiftFromStartZ = 0;
float imuVeloFromStartX = 0, imuVeloFromStartY = 0, imuVeloFromStartZ = 0;

  
void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                       float alx, float aly, float alz, float &acx, float &acy, float &acz)
{
  float sbcx = sin(bcx);
  float cbcx = cos(bcx);
  float sbcy = sin(bcy);
  float cbcy = cos(bcy);
  float sbcz = sin(bcz);
  float cbcz = cos(bcz);

  float sblx = sin(blx);
  float cblx = cos(blx);
  float sbly = sin(bly);
  float cbly = cos(bly);
  float sblz = sin(blz);
  float cblz = cos(blz);

  float salx = sin(alx);
  float calx = cos(alx);
  float saly = sin(aly);
  float caly = cos(aly);
  float salz = sin(alz);
  float calz = cos(alz);

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
            - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
            - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  acx = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
  
  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
               - calx*calz*cblx*sblz);
  acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}
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
void imuTransHandler(const sensor_msgs::PointCloud2ConstPtr& imuTrans2)
{
  timeImuTrans = imuTrans2->header.stamp.toSec();

  imuTrans->clear();
  pcl::fromROSMsg(*imuTrans2, *imuTrans);

  imuPitchStart = imuTrans->points[0].x;
  imuYawStart = imuTrans->points[0].y;
  imuRollStart = imuTrans->points[0].z;

  imuPitchLast = imuTrans->points[1].x;
  imuYawLast = imuTrans->points[1].y;
  imuRollLast = imuTrans->points[1].z;

  imuShiftFromStartX = imuTrans->points[2].x;
  imuShiftFromStartY = imuTrans->points[2].y;
  imuShiftFromStartZ = imuTrans->points[2].z;

  imuVeloFromStartX = imuTrans->points[3].x;
  imuVeloFromStartY = imuTrans->points[3].y;
  imuVeloFromStartZ = imuTrans->points[3].z;

  newImuTrans = true;
}

void paramHandler(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	reset=true;
	voxel_size=array->data[0];
	resolution=array->data[1];
}
#define velodyne
//#define genetic
//#define WITHDEBUG

lslgeneric::NDTFuserHMT *fuser;
void int_handler(int x)
{
    ROS_INFO("Saving current map to map directory ");
    fuser->saveMap(); 

}
int main(int argc, char** argv)
{
	float skipB,size_x,size_y,size_z;
	ros::init(argc, argv, "laserOdometry");
	ros::NodeHandle nh("~");
		bool beHMT=true;
		float sensor_range=30;
		bool visualize=false;
		bool doMultires=true;
		std::string map_name="toonoma";
		std::string map_dir="map";
	nh.param("voxel_size",voxel_size,0.05f);
	nh.param("size_x",size_x,50.05f);
	nh.param("size_y",size_y,50.05f);
	nh.param("size_z",size_z,5.05f);
	nh.param("sensor_range",sensor_range,30.0f);
	nh.param("resolution", resolution,1.00f);
	nh.param("numIter", numIter,35);
	nh.param("doMultires", doMultires,true);
	nh.param("visualize",visualize,true);
	signal(SIGINT,int_handler);
	std::cout<<std::endl<<std::endl<<"Running NDT  ";

    ros::Subscriber subReset = nh.subscribe<std_msgs::Float32MultiArray> ("/NDTparams", 5,paramHandler);
	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 2, laserCloudFullResHandler);
	ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2> ("/imu_trans", 5, imuTransHandler);
	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_3", 2);

  ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("dummy_occ_map_pub", 1000);

	ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
	nav_msgs::Odometry laserOdometry;
	laserOdometry.header.frame_id = "/camera_init";
	laserOdometry.child_frame_id = "/laser_odom";

	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform laserOdometryTrans;
	laserOdometryTrans.frame_id_ = "/camera_init";
	laserOdometryTrans.child_frame_id_ = "/laser_odom";


	ros::Rate rate(100);
	#ifdef genetic
		while(!reset&&ros::ok())
		{
			rate.sleep();
			ros::spinOnce();
		}
		std_msgs::Int16 fitnessMsg;
	#endif
	while(true)
	{

		fuser = new lslgeneric::NDTFuserHMT(resolution,size_x,size_y,size_z, sensor_range, visualize,false,doMultires, false, numIter, map_name, beHMT, map_dir, true);
		reset=false;
		bool systemInited = false;
	    double pose_init_x=0,pose_init_y=0,pose_init_z=0,
		  pose_init_r=0,pose_init_p=0,pose_init_t=0;
		Eigen::Affine3d pose_, Tmotion, sensor_pose_;
		pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		  Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		  Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		  Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
		Tmotion =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
		  Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
		  Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
		  Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
		Tmotion.setIdentity();
		ROS_INFO("Init pose is (%lf,%lf,%lf)", pose_.translation()(0), pose_.translation()(1), 
                 pose_.rotation().eulerAngles(0,1,2)(0));
	 
		while(ros::ok()&&!reset)
		{
			rate.sleep();
			ros::spinOnce();
			if (!newLaserCloudFullRes ) 
				continue;
			newLaserCloudFullRes = false;
			newImuTrans = false;


			if (!systemInited) 
			{
				fuser->initialize(pose_,*input_cloud.get());

				sensor_msgs::PointCloud2 interestPointsLast2;
				pcl::toROSMsg(*input_cloud, interestPointsLast2);
				interestPointsLast2.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
				interestPointsLast2.header.frame_id = "/camera";
				pubLaserCloudFullRes.publish(interestPointsLast2);

				systemInited = true;
				continue;
			}


			pose_ = fuser->update(Tmotion,*input_cloud.get());
			tf::Transform transform;
			tf::transformEigenToTF(pose_, transform);
			tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"camera_init","laser_odom"));;

			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*input_cloud, laserCloudFullRes3);
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
			laserCloudFullRes3.header.frame_id = "/camera_init";
			pubLaserCloudFullRes.publish(laserCloudFullRes3);
		}
	}
	return 0;
}
