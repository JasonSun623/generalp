#include <cmath>
#include <string>
#include <vector>
#include <sure/sure_estimator.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using std::sin;
using std::cos;
using std::atan2;

float featureSize(1.2),featureSamplingRate(0.4),normalSamplingRate(0.05),normalsScale(0.15),minimumCornerness(0.10802);

ros::Publisher pubInterestPoints;

sure::SURE_Estimator<pcl::PointXYZRGB> sureE;
sure::Configuration config;

void init()
{
    // Adjust the size of the features (in meter)
    config.setSize( featureSize); 
    // Adjust the sampling rate
    config.setSamplingRate(featureSamplingRate); 
    // Adjust the normal sampling rate
    config.setNormalSamplingRate(normalSamplingRate); 
    // Adjust the influence radius for calculating normals
    config.setNormalsScale(normalsScale); 
    // Adjust the minimum Cornerness to reduce number of features on edges
    config.setMinimumCornerness(minimumCornerness); 
    // config.setEntropyCalculationMode(sure::CROSSPRODUCTS_ALL_NORMALS_PAIRWISE); 
    // set altered configuration
	config.setOctreeExpansion(200);
    sureE.setConfig(config); 
	std::cout<<config.getSamplingLevel()<<std::endl;
	std::cout<<config.getNormalSamplingLevel()<<std::endl;
	std::cout<<config.getOctreeMinimumVolumeSize()<<std::endl;
	std::cout<<config.getOctreeExpansion()<<std::endl;
	std::cout<<config.getOctreeResolutionThreshold()<<std::endl;

}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    double timeScanCur = laserCloudMsg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*laserCloudMsg, *laserCloud);

    sureE.setInputCloud(laserCloud);
    sureE.calculateFeatures(); 
    pcl::PointCloud<pcl::InterestPoint>::Ptr features;
    features = sureE.getInterestPoints();
//    if(features->points.size()>8)
//    {
        sensor_msgs::PointCloud2 interestPointsMsg;
        pcl::toROSMsg(*features,  interestPointsMsg);
        interestPointsMsg.header.stamp = laserCloudMsg->header.stamp;
        interestPointsMsg.header.frame_id = "/camera";
		interestPointsMsg.fields[3].name="intensity";
        pubInterestPoints.publish(interestPointsMsg);
//    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh("~");
  std::string inpcloud;
  std::string outcloud;
  nh.param("featureSize", featureSize, featureSize);
  nh.param("featureSamplingRate",featureSamplingRate,featureSamplingRate);
  nh.param("normalSamplingRate",normalSamplingRate,normalSamplingRate);
  nh.param("normalsScale",normalsScale,normalsScale);
  nh.param("minimumCornerness",minimumCornerness,minimumCornerness);
  nh.param<std::string>("inputCloud",inpcloud,"/velodyne_cloud");
  nh.param<std::string>("outputCloud",outcloud,"/laser_interest");
  if(argc>1)featureSize=atof(argv[1]);
  if(argc>2)featureSamplingRate=atof(argv[2]);
  if(argc>3)normalSamplingRate=atof(argv[3]);
  if(argc>4)normalsScale=atof(argv[4]);
  if(argc>5)minimumCornerness=atof(argv[5]);
  init();

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (inpcloud.c_str(), 2, laserCloudHandler);
  pubInterestPoints = nh.advertise<sensor_msgs::PointCloud2> (outcloud.c_str(), 2);
  std::cout<<inpcloud<<" "<<outcloud<<std::endl;
  ros::spin();
  return 0;
}

