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

using namespace std;
std::vector<ros::Publisher> pubLaserCloud;
int K;
double rejectPerc;

int occluded(pcl::PointXYZI a, pcl::PointXYZI b, float d)
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

vector<size_t> sort_pointcloud(vector<float> &in)
{
	vector<size_t> idx(in.size());
	vector<size_t> idx2(in.size());
	iota(idx.begin(),idx.end(),0);
	sort(idx.begin(), idx.end(),[&in](size_t i1,size_t i2){return in[i1]<in[i2];});
	size_t j=0;
	for(auto i : idx)
	{
		idx2[i]=j;
		j++;
	}
	return idx2;
}
size_t index_selector(size_t I[],size_t number_inputs,size_t number_points)
{
	size_t minI=0,minA=I[0];
	size_t maxI=0,maxA=I[0];
	for(auto i=0;i<number_inputs;i++)
	{
		if(I[i]<=minA) { minA=I[i];minI=i; }
		if(I[i]>=maxA) { maxA=I[i];maxI=i; }
	}
	return number_points-maxA<minA?maxI+number_inputs:minI;
}
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{

	double timeScanCur = laserCloudMsg->header.stamp.toSec();
	pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);
	int cloudSize = laserCloudIn->points.size();

	pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud (laserCloudIn);
	std::vector<float> cornerness (cloudSize,-1);
	std::vector<float> intensity (cloudSize);
#define n_threads 8
    #pragma omp parallel num_threads(n_threads)
	{
        #pragma omp for
		for(int i=0;i<cloudSize;i++)
		{
			std::vector<int> pointIdxKNNSearch(K);
			std::vector<float> pointDistance(K);
			std::vector<float> diffXYZ (3,0);
			intensity[i]=laserCloudIn->points[i].intensity;
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
				float distanceOfPoint =sqrt( pow(laserCloudIn->points[i].x,2)+pow(laserCloudIn->points[i].y,2)+pow(laserCloudIn->points[i].z,2));
				if(cf)cornerness[i]=numKNN>0?sqrt(pow(diffXYZ[0],2)+pow(diffXYZ[1],2)+pow(diffXYZ[2],2))/distanceOfPoint/numKNN:-1;
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
	std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloud  (4);
	auto Icor=sort_pointcloud(cornerness);
	auto Iint=sort_pointcloud(intensity);
	int cut_off_l=(1-rejectPerc)/2*cloudSize;
	int cut_off_u=(1+rejectPerc)/2*cloudSize;
	for(int i=0;i<cloudSize;i++)
	{
		if(Icor[i]<cut_off_u&&Icor[i]>cut_off_l&&cornerness[i]==-1)
			continue;
		if(Iint[i]<cut_off_u&&Iint[i]>cut_off_l&&Icor[i]<cut_off_u&&Icor[i]>cut_off_l)
			continue;
		pcl::PointXYZI point;
		point.x=laserCloudIn->points[i].x;
		point.y=laserCloudIn->points[i].y;
		point.z=laserCloudIn->points[i].z;
		point.intensity=laserCloudIn->points[i].intensity;
		size_t k[]={Icor[i],Iint[i]};
		int index=0;
		if(cornerness[i]==-1)
			k[0]=cloudSize/2;
		laserCloud[index_selector(k,2,cloudSize)].points.push_back(point);
	}
	cout<<(float (laserCloud[0].points.size()+laserCloud[1].points.size()+laserCloud[2].points.size()+laserCloud[3].points.size()))/cloudSize*100<<endl;
	for(int i=0;i<4;i++)
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
	nh.param("reject", rejectPerc,0.5);
	pubLaserCloud.push_back(nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input0", 2)); 
	pubLaserCloud.push_back(nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input1", 2)); 
	pubLaserCloud.push_back(nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input2", 2)); 
	pubLaserCloud.push_back(nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_input3", 2)); 
	ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/kitti_player/hdl64e", 2, laserCloudHandler);
	ros::spin();
	return 0;
}

