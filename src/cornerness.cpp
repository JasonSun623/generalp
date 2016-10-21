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
#include <vector>

#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <omp.h>

float minIntL;
float minIntU;
float maxIntL;
float maxIntU;
int K;

int occluded(pcl::PointXYZ a, pcl::PointXYZ b, float d)
{
	float d1 = sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
	float d2 = sqrt(b.x*b.x+b.y*b.y+b.z*b.z);
	if(d1>d2)
	{
		float dx=b.x-a.x*d2/d1;
		float dy=b.y-a.y*d2/d1;
		float dz=b.z-a.z*d2/d1;
		if(sqrt(dx*dx+dy*dy+dz*dz)/d2<d)
			return 1;
	}
	return 0;
}

ros::Publisher pubLaserCloud;
ros::Publisher pubLaserCloud0;
ros::Publisher pubLaserCloud1;
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{

	ros::NodeHandle nh("~");
	nh.param("minIntL",minIntL,minIntL);
	nh.param("minIntU",minIntU,minIntU);
	nh.param("maxIntL",maxIntL,maxIntL);
	nh.param("maxIntU",maxIntU,maxIntU);
	nh.param("K",K,K);


	double timeScanCur = laserCloudMsg->header.stamp.toSec();
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
	int cloudSize = laserCloudIn->points.size();

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (laserCloudIn);
	std::vector<float> cornerness (cloudSize,-1);
#define n_threads 6
    #pragma omp parallel num_threads(n_threads)
	{
        #pragma omp for
		for(int i=0;i<cloudSize;i++)
		{
			std::vector<int> pointIdxKNNSearch(K);
			std::vector<float> pointDistance(K);
			std::vector<float> diffXYZ (3,0);
			if(kdtree.nearestKSearch(laserCloudIn->points[i],K,pointIdxKNNSearch,pointDistance)>0)
			{
				int minCornIndex=i;
				int maxCornIndex=i;
				bool cf=1;
				int numKNN=pointIdxKNNSearch.size();
				for(int j=0;j<numKNN;++j)
				{
					diffXYZ[0]+=laserCloudIn->points[pointIdxKNNSearch[j]].x-laserCloudIn->points[i].x;
					diffXYZ[1]+=laserCloudIn->points[pointIdxKNNSearch[j]].y-laserCloudIn->points[i].y;
					diffXYZ[2]+=laserCloudIn->points[pointIdxKNNSearch[j]].z-laserCloudIn->points[i].z;
					float disOcc=0.2;
					if(pointDistance[j]>disOcc&&
							occluded(laserCloudIn->points[i],laserCloudIn->points[pointIdxKNNSearch[j]],disOcc))
					{
						cf=0;
						break;
					}
					if(cornerness[j]<cornerness[minCornIndex]||cornerness[minCornIndex]==-1)
						minCornIndex = j;
					else if(cornerness[j]>cornerness[maxCornIndex])
						maxCornIndex = j;
					else cornerness[j]=-1;
				}
				float distanceOfPoint = pow(laserCloudIn->points[i].x,2)+pow(laserCloudIn->points[i].y,2)+pow(laserCloudIn->points[i].z,2);
				if(cf)cornerness[i]=sqrt(pow(diffXYZ[0],2)+pow(diffXYZ[1],2)+pow(diffXYZ[2],2));
				if(cornerness[i]!=-1)
				{
					if(cornerness[i]<cornerness[minCornIndex]&&minCornIndex!=i)
						cornerness[minCornIndex]=-1;
					else if(cornerness[i]>cornerness[maxCornIndex]&&maxCornIndex!=i)
						cornerness[maxCornIndex]=-1;
					else if (maxCornIndex!=i&&minCornIndex!=i)cornerness[i]=-1;
				}
			}
		}
	}
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud (new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0;i<cloudSize;i++)
		if(cornerness[i]!=-1)
			laserCloud->push_back(laserCloudIn->points[i]);
			*/
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud  (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud0 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud1 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointXYZI point;

    #pragma omp parallel num_threads(n_threads)
	{
        #pragma omp for private(point)
		for(int i=0;i<cloudSize;i++)
		{
			point.x=laserCloudIn->points[i].x;
			point.y=laserCloudIn->points[i].y;
			point.z=laserCloudIn->points[i].z;
			point.intensity=cornerness[i];
			if(point.intensity>0)
#pragma omp critical(dataupdate)
			{
				laserCloud->points.push_back(point);
			}
			if(point.intensity<minIntU&&point.intensity>minIntL)
#pragma omp critical(dataupdate)
			{
				laserCloud0->points.push_back(point);
			}
			else if(point.intensity<maxIntU&&point.intensity>maxIntL)
#pragma omp critical(dataupdate)
			{
				laserCloud1->points.push_back(point);
			}
		}
	}
 
  sensor_msgs::PointCloud2 laserCloudOutMsg;
  pcl::toROSMsg(*laserCloud0, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/camera";
  pubLaserCloud0.publish(laserCloudOutMsg);
  printf("%c[2K\r",27);
  std::cout<< " "<<((float ) laserCloudIn->points.size()-(float )(laserCloud1->points.size()+laserCloud0->points.size()))/laserCloudIn->points.size()<<"\t\t"<<std::flush;
  printf("\r");
            
  sensor_msgs::PointCloud2 laserCloudOutMsg1;
  pcl::toROSMsg(*laserCloud1, laserCloudOutMsg1);
  laserCloudOutMsg1.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg1.header.frame_id = "/camera";
  pubLaserCloud1.publish(laserCloudOutMsg1);
            
  sensor_msgs::PointCloud2 laserCloudOutMsg2;
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg2);
  laserCloudOutMsg2.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg2.header.frame_id = "/camera";
  pubLaserCloud.publish(laserCloudOutMsg2);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh("~");
	nh.param("minIntL",minIntL,0.00f);
	nh.param("minIntU",minIntU,0.1f);
	nh.param("maxIntL",maxIntL,0.8f);
	nh.param("maxIntU",maxIntU,2.05f);
	nh.param("K",K,10);

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/kitti_player/hdl64e", 2, laserCloudHandler);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input", 2); 
  pubLaserCloud0 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input0", 2); 
  pubLaserCloud1 = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input1", 2); 

  ros::spin();

  return 0;
}

