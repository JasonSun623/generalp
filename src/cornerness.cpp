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

std::vector<boost::array<float,2> > limits;
std::vector<ros::Publisher> pubLaserCloud;
int numRegions;
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

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{

	ros::NodeHandle nh("~");
	int i=0;
	while(nh.getParam(("limLower"+boost::lexical_cast<std::string>(i)).c_str(),limits[i][0]) && nh.getParam(("limUpper"+boost::lexical_cast<std::string>(i)).c_str(),limits[i][1]) && ++i);

	double timeScanCur = laserCloudMsg->header.stamp.toSec();
	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
	int cloudSize = laserCloudIn->points.size();

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (laserCloudIn);
	std::vector<float> cornerness (cloudSize,-1);
#define n_threads 8
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
	std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloud  (numRegions+1);
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
				laserCloud[0].points.push_back(point);
			}
			for(int j=0;j<numRegions;j++)
				if(point.intensity<limits[j][1]&&point.intensity>limits[j][0])
#pragma omp critical(dataupdate)
				{
					laserCloud[j+1].points.push_back(point);
				}
		}
	}

	for(int i=0;i<numRegions+1;i++)
	{
			
		  sensor_msgs::PointCloud2 laserCloudOutMsg;
		  pcl::toROSMsg(laserCloud[i], laserCloudOutMsg);
		  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
		  laserCloudOutMsg.header.frame_id = "/camera";
		  pubLaserCloud[i].publish(laserCloudOutMsg);
	}
}
int main(int argc, char** argv)
{

  	ros::init(argc, argv, "cornerness");
  	ros::NodeHandle nh("~");
	nh.param("K",K,10);
    pubLaserCloud.push_back(nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input", 2)); 
	int i=0;
limits.push_back({0.0,0.0} ); 
	while(nh.getParam(("limLower"+boost::lexical_cast<std::string>(i)).c_str(),limits[i][0]) && nh.getParam(("limUpper"+boost::lexical_cast<std::string>(i)).c_str(),limits[i][1]))
	{
		limits.push_back({0.0,0.0} );
		pubLaserCloud.push_back(nh.advertise<sensor_msgs::PointCloud2> (("/velodyne_input"+boost::lexical_cast<std::string>(i)).c_str(), 2)); 
		i++;
	}
	limits.pop_back();
	numRegions=limits.size();
	ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/kitti_player/hdl64e", 2, laserCloudHandler);
	ros::spin();
	return 0;
}

