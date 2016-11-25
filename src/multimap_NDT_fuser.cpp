#include <cmath>
#include <unistd.h>
#include <cstdio>

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
#include <tf_conversions/tf_eigen.h>
#include <csignal>
#include <ndt_map/ndt_conversions.h>
#include <ndt_fuser/ndt_fuser_hmt_l.h>
using namespace std;
void run();
Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> T;
double timeLaserCloudFullRes = 0;

double startOri;
double endOri;
	int NumInputs=0;
char rotatingFun[4]={'|','/','-','\\'};
int rotateCount=0;
void transformPointCloudInPlaceRectified(Eigen::Transform<double,3,Eigen::Affine,Eigen::ColMajor> Tr, pcl::PointCloud<pcl::PointXYZ> *pc)
{
	double angle2;
	angle2=startOri-endOri;
	angle2+=angle2<2.0944?6.2832:0;
			//cout<<"Translation: "<<Tr.translation()<<endl;
			//cout<<"Rotation: " <<Tr.rotation()<<endl;
	for(unsigned int jit=0;jit<NumInputs;jit++)
	{
		for(unsigned int pit=0; pit<pc[jit].points.size(); ++pit)
		{
			Eigen::Map<Eigen::Vector3f> pt((float*)&pc[jit].points[pit],3);
			double angle;
			angle =startOri -atan2(pt[1],pt[0]);
			if(angle<0)angle=3.1416-angle;
			Eigen::Transform<float,3,Eigen::Affine,Eigen::ColMajor> T = Tr.cast<float>();
			for(int i=0;i<3;i++)
				for(int j=0;j<2;j++)
					T(i,j)=T(i,j)*angle/angle2;
			pt = T *pt;
		}
	}
}
class input_cloud_handler{
	pcl::PointCloud<pcl::PointXYZ> *input_cloud ;
	public:
	int num;
	float voxel_size;
	unsigned int *newCloud;
	input_cloud_handler(unsigned int *newCloudt){voxel_size=0;num=0;newCloud=newCloudt;};
	input_cloud_handler(unsigned int *newCloudt,float voxel_size1,int num1,pcl::PointCloud<pcl::PointXYZ> *tmpC)
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
void angleCallback(const std_msgs::Float32MultiArray::ConstPtr& floats)
{
	startOri=floats->data[0];
	endOri=floats->data[1];
}
class cloud_handlers{
	unsigned int N;
	unsigned int newCloud;
	public:
	pcl::PointCloud<pcl::PointXYZ> *input_cloud;
		std::vector<input_cloud_handler> handler;
		cloud_handlers(int Nt,float *voxel_size)
		{
			newCloud=0;
			N=Nt;
			input_cloud=new pcl::PointCloud<pcl::PointXYZ>[N];
			for(int i=0;i<N;i++)
				handler.push_back(input_cloud_handler(&newCloud,voxel_size[i],i,&input_cloud[i]));
		}
		bool check_new(){
			if(__builtin_popcount(newCloud)==N)
			{
				newCloud=0;
				return true;
			}else return false;
		}
};
bool term=false;
void int_handler(int x)
{
	term=true;
}
int main(int argc, char** argv)
{
	float skipB,size_x,size_y,size_z;
	//signal(SIGINT,int_handler);
	ros::init(argc, argv, "ndt_registration");
	ros::NodeHandle nh("~");
	float sensor_range;
	int numIter;
	bool with_odom=false;

	int maxInputs=20;

	float *resolution = new float[maxInputs];
	float *voxel_size = new float[maxInputs];
	char * input_pipe = "/tmp/multimap_fifo";
	mkfifo(input_pipe,0666);
	std::ifstream fifo;
	fifo.open(input_pipe,ifstream::in);

	while(nh.getParam(("voxel_size"+boost::lexical_cast<std::string>(NumInputs)).c_str(),voxel_size[NumInputs]) && nh.getParam(("resolution"+boost::lexical_cast<std::string>(NumInputs)).c_str(),resolution[NumInputs])&& ++NumInputs);

	nh.param("size_x",size_x,80.05f);
	nh.param("size_y",size_y,80.05f);
	nh.param("size_z",size_z,15.05f);
	nh.param("sensor_range",sensor_range,100.3f);
	nh.param("numIter", numIter,150);
	nh.param("with_odometry",with_odom,false);

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

	ros::Subscriber angleSubscribe = nh.subscribe<std_msgs::Float32MultiArray>("/start_end_angles",1,angleCallback);

	ros::Rate rate(10);
	bool systemInited = false;
	double pose_init_x=0,pose_init_y=0,pose_init_z=0,
	  pose_init_r=0,pose_init_p=0,pose_init_t=0;
	nh.param("pose_init_r", pose_init_r,pose_init_r);
	nh.param("pose_init_p", pose_init_p,pose_init_p);
	nh.param("pose_init_t", pose_init_t,pose_init_t);
	Eigen::Affine3d pose_, Tinit;
	pose_ =  Eigen::Translation<double,3>(pose_init_x,pose_init_y,pose_init_z)*
	  Eigen::AngleAxis<double>(pose_init_r,Eigen::Vector3d::UnitX()) *
	  Eigen::AngleAxis<double>(pose_init_p,Eigen::Vector3d::UnitY()) *
	  Eigen::AngleAxis<double>(pose_init_t,Eigen::Vector3d::UnitZ()) ;
	Tinit.setIdentity();
	ROS_INFO("Init pose is (%lf,%lf,%lf)", pose_.translation()(0), pose_.translation()(1), pose_.rotation().eulerAngles(0,1,2)(0));

	lslgeneric::NDTFuserHMT_L map_fuser (NumInputs,resolution,size_x,size_y,size_z,sensor_range,false,false,30,"maps",true,"map",true);

	Eigen::Affine3d to_cor=pose_.inverse();

	tf::StampedTransform tf_c;
	Eigen::Affine3d tf_now,tf_before;
	tf::TransformListener tf_listener;

	std::cout<<std::endl<<std::endl<<"Running NDT  "<<std::endl;
	int numB;
	while(!term&&ros::ok())
	{
		rate.sleep();
		ros::spinOnce();
		std::string word;
		if(fifo>>word)
			map_fuser.saveMap();
		if(fifo.eof())
			fifo.clear();
		if (!input_clouds.check_new()) 
			continue;
		if(!systemInited)
		{
			map_fuser.initialize(pose_,input_clouds.input_cloud,false);
			systemInited=true;
			continue;
		}
		Tinit.setIdentity();
		if(with_odom)
		{
			tf_before=tf_now;
			tf_listener.lookupTransform("odom","base_link",ros::Time(0),tf_c);
			tf::poseTFToEigen(tf_c,tf_now);
			Eigen::Transform<double,3,Eigen::Affine> odom_trans;
			odom_trans.setIdentity();
			odom_trans*=(tf_now.rotation() -tf_before.rotation());
			odom_trans.translate(tf_now.translation() -tf_before.translation());
			Tinit.rotate(odom_trans.rotation());
			Tinit.translate(odom_trans.translation());
		}
		pose_=map_fuser.update(Tinit,input_clouds.input_cloud);

		tf::Transform transform;
		tf::transformEigenToTF(to_cor*pose_, transform);
		tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"camera_init","camera"));;

		tf::Transform transform2;
		tf::transformEigenToTF(pose_, transform2);
		tfBroadcaster2.sendTransform(tf::StampedTransform(transform2, ros::Time::now(),"camera_init","camera2"));;

		pubLaserOdometry.publish(laserOdometry);

	}
	map_fuser.saveMap();
	return 0;
}

