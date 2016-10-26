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
using namespace std;
void run();
Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
double timeLaserCloudFullRes = 0;

char rotatingFun[4]={'|','/','-','\\'};
int rotateCount=0;

class input_cloud_handler{
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud ;
	public:
	int num;
	float voxel_size;
	unsigned int *newCloud;
	input_cloud_handler(unsigned int *newCloudt){voxel_size=0;num=0;newCloud=newCloudt;};
	input_cloud_handler(unsigned int *newCloudt,float voxel_size1,int num1,pcl::PointCloud<pcl::PointXYZ>::Ptr tmpC)
	{
		voxel_size=voxel_size1;
		num=num1;
		newCloud=newCloudt;
		input_cloud=tmpC;
	};
	void laserHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud)
	{
		//std::cout << '\b'<<rotatingFun[++rotateCount%4] << std::flush;
		pcl::PointCloud<PointType>::Ptr laserCloudTemp(new pcl::PointCloud<PointType>);

		timeLaserCloudFullRes = laserCloud->header.stamp.toSec();
		pcl::fromROSMsg(*laserCloud, *laserCloudTemp);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*laserCloudTemp,*laserCloudTemp, indices);
		if(voxel_size!=0)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr non_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			copyPointCloud(*laserCloudTemp,*non_filtered_cloud);
			pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
			approximate_voxel_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
			approximate_voxel_filter.setInputCloud (non_filtered_cloud);
			approximate_voxel_filter.filter (*input_cloud);
		}
		else
			copyPointCloud(*laserCloudTemp,*input_cloud);
		*newCloud|=1<<num;
	}
};

class allMaps{
	public:
	std::vector<lslgeneric::NDTMap > mapG ;
	std::vector<lslgeneric::NDTMap > mapL;
	void add_map(float resolution)
	{
		lslgeneric::LazyGrid *tmp1 = (lslgeneric::LazyGrid*) malloc(2*sizeof(lslgeneric::LazyGrid));
		new(tmp1) lslgeneric::LazyGrid(resolution);
		lslgeneric::LazyGrid *tmp2 = (lslgeneric::LazyGrid*) malloc(2*sizeof(lslgeneric::LazyGrid));
		new(tmp2) lslgeneric::LazyGrid(resolution);
		lslgeneric::NDTMap *tmp_map1 = (lslgeneric::NDTMap*) malloc(2*sizeof(lslgeneric::NDTMap));
		new(tmp_map1) lslgeneric::NDTMap(tmp1);
		lslgeneric::NDTMap *tmp_map2 = (lslgeneric::NDTMap*) malloc(2*sizeof(lslgeneric::NDTMap));
		new(tmp_map2) lslgeneric::NDTMap(tmp2);
		mapG.push_back(*tmp_map1);
		mapL.push_back(*tmp_map2);
	}
};
class cloud_handlers{
	unsigned int N;
	unsigned int newCloud;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > input_cloud;
	public:
		std::vector<input_cloud_handler> handler;
		cloud_handlers(int Nt,float *voxel_size)
		{
			newCloud=0;
			N=Nt;
			for(int i=0;i<N;i++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZ>) ;
				input_cloud.push_back(tmp);
				handler.push_back(input_cloud_handler(&newCloud,voxel_size[i],i,tmp));
			}
		}
		bool check_new(){
			if(__builtin_popcount(newCloud)==N)
			{
				newCloud=0;
				return true;
			}else return false;
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr operator[](std::size_t idx) {return input_cloud[idx];};
};

int main(int argc, char** argv)
{
	float skipB,size_x,size_y,size_z;
	ros::init(argc, argv, "ndt_registration");
	ros::NodeHandle nh("~");
	float sensor_range;
	int numIter;

	int maxInputs=20;
	int NumInputs=0;

	float *resolution = new float[maxInputs];
	float *voxel_size = new float[maxInputs];

	while(nh.getParam(("voxel_size"+boost::lexical_cast<std::string>(NumInputs)).c_str(),voxel_size[NumInputs]) && nh.getParam(("resolution"+boost::lexical_cast<std::string>(NumInputs)).c_str(),resolution[NumInputs])&& ++NumInputs);

	nh.param("size_x",size_x,80.05f);
	nh.param("size_y",size_y,80.05f);
	nh.param("size_z",size_z,15.05f);
	nh.param("sensor_range",sensor_range,100.3f);
	nh.param("numIter", numIter,150);

	cloud_handlers input_clouds (NumInputs,voxel_size);
	ros::Subscriber *subscribers = new ros::Subscriber[NumInputs];
	for(int i=0;i<NumInputs;i++)
	{
		string line="/velodyne_input"+boost::lexical_cast<std::string>(i);
		subscribers[i] = nh.subscribe<sensor_msgs::PointCloud2> (line.c_str(), 1, &input_cloud_handler::laserHandler,&input_clouds.handler[i] );
		cout<<i<<endl;
	}

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
	ROS_INFO("Init pose is (%lf,%lf,%lf)", pose_.translation()(0), pose_.translation()(1), pose_.rotation().eulerAngles(0,1,2)(0));

	lslgeneric::NDTMatcherD2DL matcher;
	matcher.NumInputs=NumInputs;
	matcher.ITR_MAX =numIter;
	matcher.step_control=true;

	lslgeneric::NDTMap *mapG[100];
	lslgeneric::NDTMap *mapL[100];

	for(unsigned int i=0;i< NumInputs;i++)
	{
		lslgeneric::LazyGrid *grid0 = new lslgeneric::LazyGrid(resolution[i]);
		lslgeneric::NDTMap *map0 = new lslgeneric::NDTMap(grid0);
		lslgeneric::LazyGrid *grid1 = new lslgeneric::LazyGrid(resolution[i]);
		lslgeneric::NDTMap *map1 = new lslgeneric::NDTMap(grid1);
		mapG[i]=map0;
		mapL[i]=map1;
		mapG[i]->guessSize(0,0,0,size_x,size_y,size_z);
		mapL[i]->guessSize(0,0,0,size_x,size_y,size_z);
		mapG[i]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
		mapL[i]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
	}	
	Eigen::Affine3d to_cor=pose_.inverse();

	std::cout<<std::endl<<std::endl<<"Running NDT  "<<std::endl;
//	return 0;
//	mutex.unlock();
	while(ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
		if (!input_clouds.check_new()) 
			continue;

		if(systemInited)
		{
			for(unsigned int i=0;i<NumInputs;i++)
			{
				mapL[i]->loadPointCloud(*input_clouds[i],sensor_range);
				mapL[i]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
			}
			matcher.match(mapG,mapL,Tinit,true) ;
		}
		systemInited=true;
		pose_=pose_*Tinit;
		tf::Transform transform;
		tf::transformEigenToTF(to_cor*pose_, transform);
		tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"camera_init","camera"));;

		tf::Transform transform2;
		tf::transformEigenToTF(pose_, transform2);
		tfBroadcaster2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(),"camera_init","camera2"));;

		for(unsigned int i=0;i<NumInputs;i++)
		{
			mapG[i]->loadPointCloud(*input_clouds[i],sensor_range);
			mapG[i]->computeNDTCells(CELL_UPDATE_MODE_SAMPLE_VARIANCE);
		}
		pubLaserOdometry.publish(laserOdometry);

	}
}

