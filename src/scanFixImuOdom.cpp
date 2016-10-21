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
//
//
//
//
//
//
//
//     IMPORTANT NOTE
//     currently no fusing between signals of odometry and IMU. If both signals are available
//     the calculation will fail. Needs adaptation

#include <cmath>
#include <vector>

#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::sin;
using std::cos;
using std::atan2;

const double scanPeriod = 0.1;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

const int N_SCANS = 16;

int cloudLabel[40000];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

ros::Publisher pubLaserCloud;
ros::Publisher pubImuTrans;

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

void init()
{
    systemInitCount = 0;
    systemInited = false;

    imuPointerFront = 0;
    imuPointerLast = -1;

    imuRollStart = 0; imuPitchStart = 0; imuYawStart = 0;
    imuRollCur = 0; imuPitchCur = 0; imuYawCur = 0;

    imuVeloXStart = 0; imuVeloYStart = 0; imuVeloZStart = 0;
    imuShiftXStart = 0; imuShiftYStart = 0; imuShiftZStart = 0;

    imuVeloXCur = 0; imuVeloYCur = 0; imuVeloZCur = 0;
    imuShiftXCur = 0; imuShiftYCur = 0; imuShiftZCur = 0;

    imuShiftFromStartXCur = 0; imuShiftFromStartYCur = 0; imuShiftFromStartZCur = 0;
    imuVeloFromStartXCur = 0; imuVeloFromStartYCur = 0; imuVeloFromStartZCur = 0;

    std::fill(imuTime, imuTime+imuQueLength-1,0);
    std::fill(imuRoll,imuRoll+imuQueLength-1,0);
    std::fill(imuPitch,imuPitch+imuQueLength-1,0);
    std::fill(imuYaw,imuYaw+imuQueLength-1,0);
    std::fill(imuAccX,imuAccX+imuQueLength-1,0);
    std::fill(imuAccY,imuAccY+imuQueLength-1,0);
    std::fill(imuAccZ,imuAccZ+imuQueLength-1,0);
    std::fill(imuVeloX,imuVeloX+imuQueLength-1,0);
    std::fill(imuVeloY,imuVeloY+imuQueLength-1,0);
    std::fill(imuVeloZ,imuVeloZ+imuQueLength-1,0);
    std::fill(imuShiftX,imuShiftX+imuQueLength-1,0);
    std::fill(imuShiftY,imuShiftY+imuQueLength-1,0);
    std::fill(imuShiftZ,imuShiftZ+imuQueLength-1,0);

}

void ShiftToStartIMU(float pointTime)
{
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

void VeloToStartIMU()
{
  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}

template <typename PointT>
void TransformToStartIMU(PointT *p)
{
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;
  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < scanPeriod) {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    if (!systemInited) 
	{
		systemInitCount++;
		if (systemInitCount >= systemDelay) 
			systemInited = true;
    return;
	}

    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    double timeScanCur = laserCloudMsg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    int cloudSize = laserCloudIn.points.size();
	if(cloudSize<1)return;
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y, laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }
    bool halfPassed = false;
    int count = cloudSize;
    pcl::PointXYZRGB point;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB> > laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++) {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].z;
        point.z = laserCloudIn.points[i].y;

        float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
        int scanID;
        int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5)); 
        if (roundedAngle > 0) scanID = roundedAngle;
        else scanID = roundedAngle + (N_SCANS - 1);
        if (scanID > (N_SCANS - 1) || scanID < 0 ){
            count--;
            continue;
        }

        float ori = -atan2(point.x, point.z);
        if (!halfPassed) {
          if (ori < startOri - M_PI / 2) ori += 2 * M_PI;
          else if (ori > startOri + M_PI * 3 / 2) ori -= 2 * M_PI; 
          if (ori - startOri > M_PI) halfPassed = true;
        } 
        else 
        {
          ori += 2 * M_PI;
          if (ori < endOri - M_PI * 3 / 2) 
            ori += 2 * M_PI;
          else if (ori > endOri + M_PI / 2)
            ori -= 2 * M_PI;
        }

        float relTime = (ori - startOri) / (endOri - startOri);
        point.rgb = scanID + scanPeriod * relTime;
        if (imuPointerLast >= 0) {
          float pointTime = relTime * scanPeriod;
          while (imuPointerFront != imuPointerLast) {
            if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
              break;
            }
            imuPointerFront = (imuPointerFront + 1) % imuQueLength;
          }

          if (timeScanCur + pointTime > imuTime[imuPointerFront])
          {
            imuRollCur = imuRoll[imuPointerFront];
            imuPitchCur = imuPitch[imuPointerFront];
            imuYawCur = imuYaw[imuPointerFront];

            imuVeloXCur = imuVeloX[imuPointerFront];
            imuVeloYCur = imuVeloY[imuPointerFront];
            imuVeloZCur = imuVeloZ[imuPointerFront];

            imuShiftXCur = imuShiftX[imuPointerFront];
            imuShiftYCur = imuShiftY[imuPointerFront];
            imuShiftZCur = imuShiftZ[imuPointerFront];
          } 
          else 
          {
            int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
            float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                             / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                            / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

            imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
            imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
            if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
              imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
            } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
              imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
            } else {
              imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
            }

            imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
            imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
            imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

            imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
            imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
            imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
          }
          if (i == 0) {
            imuRollStart = imuRollCur;
            imuPitchStart = imuPitchCur;
            imuYawStart = imuYawCur;

            imuVeloXStart = imuVeloXCur;
            imuVeloYStart = imuVeloYCur;
            imuVeloZStart = imuVeloZCur;

            imuShiftXStart = imuShiftXCur;
            imuShiftYStart = imuShiftYCur;
            imuShiftZStart = imuShiftZCur;
          } else {
            ShiftToStartIMU(pointTime);
            VeloToStartIMU();
            TransformToStartIMU(&point);
          }
        }
        laserCloudScans[scanID].push_back(point);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int i = 0; i < N_SCANS; i++) {
        *laserCloud += laserCloudScans[i];
    }
    int scanCount = -1;


////std::cout<<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<std::endl;
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/camera";
    laserCloudOutMsg.fields[3].name="intensity";
    pubLaserCloud.publish(laserCloudOutMsg);

/////std::cout<<"++++++++++++++++++++++++++++++"<<std::endl;

    pcl::PointCloud<pcl::PointXYZ> imuTrans(4, 1);
    imuTrans.points[0].x = imuPitchStart;
    imuTrans.points[0].y = imuYawStart;
    imuTrans.points[0].z = imuRollStart;

    imuTrans.points[1].x = imuPitchCur;
    imuTrans.points[1].y = imuYawCur;
    imuTrans.points[1].z = imuRollCur;

    imuTrans.points[2].x = imuShiftFromStartXCur;
    imuTrans.points[2].y = imuShiftFromStartYCur;
    imuTrans.points[2].z = imuShiftFromStartZCur;

    imuTrans.points[3].x = imuVeloFromStartXCur;
    imuTrans.points[3].y = imuVeloFromStartYCur;
    imuTrans.points[3].z = imuVeloFromStartZCur;

///std::cout<<"************************************************"<<std::endl;
    sensor_msgs::PointCloud2 imuTransMsg;
    pcl::toROSMsg(imuTrans, imuTransMsg);
    imuTransMsg.header.stamp = laserCloudMsg->header.stamp;
    imuTransMsg.header.frame_id = "/camera";
    pubImuTrans.publish(imuTransMsg);
///std::cout<<"------------------------------------------------"<<std::endl;
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  AccumulateIMUShift();
}
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryIn)
{
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(odometryIn->pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;
    imuTime[imuPointerLast] = odometryIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;
    imuVeloX[imuPointerLast] = odometryIn->twist.twist.linear.y ;
    imuVeloY[imuPointerLast] = odometryIn->twist.twist.linear.z ;
    imuVeloZ[imuPointerLast] = odometryIn->twist.twist.linear.x ;

    float velX=imuVeloX[imuPointerLast];
    float velY=imuVeloY[imuPointerLast];
    float velZ=imuVeloZ[imuPointerLast];
    
    imuShiftX[imuPointerLast] = odometryIn->pose.pose.position.y;
    imuShiftY[imuPointerLast] = odometryIn->pose.pose.position.z;
    imuShiftZ[imuPointerLast] = odometryIn->pose.pose.position.x;
  
      float x1 = cos(roll) * velX - sin(roll) * velY;
      float y1 = sin(roll) * velX + cos(roll) * velY;
      float z1 = velZ;

      float x2 = x1;
      float y2 = cos(pitch) * y1 - sin(pitch) * z1;
      float z2 = sin(pitch) * y1 + cos(pitch) * z1;

      velX = cos(yaw) * x2 + sin(yaw) * z2;
      velY = y2;
      velZ = -sin(yaw) * x2 + cos(yaw) * z2;

      int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
      double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
      if (timeDiff < scanPeriod) {

        imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff;
        imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff;
        imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff;

        imuVeloX[imuPointerLast] = velX;
        imuVeloY[imuPointerLast] = velY;
        imuVeloZ[imuPointerLast] = velZ;
      }    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> ("/velodyne_points", 2, laserCloudHandler);
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> ("/pose", 50, odometryHandler);
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_2", 2);
  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);



  pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);

  ros::spin();

  return 0;
}

