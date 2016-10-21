// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

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
#include <pcl/registration/icp.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>



void run();


float Grid_res,epsilon,stepSize;
int numIter;
double timeLaserCloudFullRes = 0;
double timeImuTrans = 0;

float voxel_size;
bool newLaserCloudFullRes = false;
bool newImuTrans = false;
bool reset=false;

pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
	Grid_res=array->data[1];
	epsilon=array->data[2];
	stepSize=array->data[3];
	//numIter=floor(array->data[1]);

}
#define velodyne
//#define genetic
#define WITHDEBUG

int main(int argc, char** argv)
{
	float skipB;
	ros::init(argc, argv, "laserOdometry");
	ros::NodeHandle nh("~");
	nh.param("voxel_size",voxel_size,1.34f);
	nh.param("Grid_res", Grid_res,5.51f);
	nh.param("epsilon",epsilon,0.65f);
	nh.param("stepSize",stepSize,2.24f);
	nh.param("numIter", numIter,35);
	nh.param("skipBelow", skipB,0.0f);
	std::cout<<std::endl<<std::endl<<"Running NDT  ";


    ros::Subscriber subReset = nh.subscribe<std_msgs::Float32MultiArray> ("/NDTparams", 5,paramHandler);


	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
		("/velodyne_cloud_2", 2, laserCloudFullResHandler);

	ros::Subscriber subImuTrans = nh.subscribe<sensor_msgs::PointCloud2> ("/imu_trans", 5, imuTransHandler);


	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_3", 2);


	ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/laser_odom_to_init", 5);
	nav_msgs::Odometry laserOdometry;
	laserOdometry.header.frame_id = "/camera_init";
	laserOdometry.child_frame_id = "/camera";

	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform laserOdometryTrans;
	laserOdometryTrans.frame_id_ = "/camera_init";
	laserOdometryTrans.child_frame_id_ = "/camera";


	ros::Rate rate(100);
	Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (0, 0, 0);
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
		reset=false;
		bool systemInited = false;
		Eigen::Matrix4f T= (init_translation * init_rotation).matrix ();
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
				pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudTemp = input_cloud;
				input_cloud = target_cloud;
				target_cloud = laserCloudTemp;

				sensor_msgs::PointCloud2 interestPointsLast2;
				pcl::toROSMsg(*target_cloud, interestPointsLast2);
				interestPointsLast2.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
				interestPointsLast2.header.frame_id = "/camera";
				pubLaserCloudFullRes.publish(interestPointsLast2);

				systemInited = true;
				continue;
			}

			///////NDT
			///copyPointCloud(*laserCloudFullRes,*input_cloud);
			// Filtering input scan to roughly 10% of original size to increase speed of registration.

			// Initializing Normal Distributions Transform (NDT).
			pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

			// Setting scale dependent NDT parameters
			// Setting minimum transformation difference for termination condition.
			ndt.setTransformationEpsilon (epsilon);
			// Setting maximum step size for More-Thuente line search.
			ndt.setStepSize (stepSize);
			//Setting Resolution of NDT grid structure (VoxelGridCovariance).
			ndt.setResolution (Grid_res);

			// Setting max number of registration iterations.
			ndt.setMaximumIterations (numIter);

			// Setting point cloud to be aligned.
			ndt.setInputSource (input_cloud);
			// Setting point cloud to be aligned to.
			ndt.setInputTarget (target_cloud);

			// Set initial alignment estimate found using robot odometry.
			//Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
			//Eigen::Translation3f init_translation (1.79387, 0.720047, 0);

			// Calculating required rigid transform to align the input cloud to the target cloud.
			pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			//ndt.align (*output_cloud, init_guess);
			ndt.align (*output_cloud,T);
			//ndt.align (*output_cloud);
			#ifdef  WITHDEBUG
				std::cout <<output_cloud->size()<< "NDT has converged:" << ndt.hasConverged ()
				<< " score: " << ndt.getFitnessScore () << std::endl;
				std::cout<<T<<std::endl;
			#endif
			// Transforming unfiltered, input cloud using found transform.
			//pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());
			(*target_cloud)=(*output_cloud);



			if(skipB!=0&&skipB<ndt.getFitnessScore())
				continue;



			//Eigen::Matrix4f T=icp.getFinalTransformation();
			T = ndt.getFinalTransformation();

			tf::Matrix3x3 tf3d;
			tf3d.setValue(
				static_cast<double>(T(0,0)), static_cast<double>(T(0,1)), static_cast<double>(T(0,2)), 
				static_cast<double>(T(1,0)), static_cast<double>(T(1,1)), static_cast<double>(T(1,2)), 
				static_cast<double>(T(2,0)), static_cast<double>(T(2,1)), static_cast<double>(T(2,2)));

			tf::Quaternion tfqt;
			tf::Vector3 origin;
			origin.setValue(
				static_cast<double>(T(0,3)),
				static_cast<double>(T(1,3)),
				static_cast<double>(T(2,3)));
			tf3d.getRotation(tfqt);
			laserOdometryTrans.setOrigin(origin);
			laserOdometryTrans.setRotation(tfqt);
			laserOdometryTrans.stamp_ = ros::Time().fromSec(timeLaserCloudFullRes );
			tfBroadcaster.sendTransform(laserOdometryTrans);

			double rxd, ryd, rzd;
			float tx, ty, tz;
			//float rx, ry, rz, tx, ty, tz;
			tf3d.getRPY(rxd,ryd,rzd);
			tx= static_cast<float>(T(0,3));
			ty= static_cast<float>(T(1,3));
			tz= static_cast<float>(T(2,3));


			geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rxd, ryd, rzd);

			laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes );
			laserOdometry.pose.pose.orientation.x = geoQuat.x;
			laserOdometry.pose.pose.orientation.y = geoQuat.y;
			laserOdometry.pose.pose.orientation.z = geoQuat.z;
			laserOdometry.pose.pose.orientation.w = geoQuat.w;
			laserOdometry.pose.pose.position.x = tx;
			laserOdometry.pose.pose.position.y = ty;
			laserOdometry.pose.pose.position.z = tz;
			pubLaserOdometry.publish(laserOdometry);


			sensor_msgs::PointCloud2 laserCloudFullRes3;
			pcl::toROSMsg(*output_cloud, laserCloudFullRes3);
			laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserCloudFullRes);
			laserCloudFullRes3.header.frame_id = "/camera_init";
			pubLaserCloudFullRes.publish(laserCloudFullRes3);
		}
	}
	return 0;
}
