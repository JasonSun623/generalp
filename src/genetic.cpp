#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "rosbag/bag.h"
#include "rosbag/player.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <unistd.h>
#include "std_msgs/Int16.h"
#include <sys/types.h>
#include <signal.h>
#include <wait.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <unistd.h>


int numParams,numIter;
bool ConfComp(float *i1, float *i2){return (i1[numParams]<=i2[numParams]);};


#define POP_LEN 15
#define RET_POP 5

float poseDist=0;
float pointRc[3]={0,0,0};
float pointRp[3]={0,0,0};
float pointT[3]={0,0,0};
bool initial_pose_set=false;
float errorSum=0;
int numSample;

void refHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::Point cur = odom->pose.pose.position;
	//std::cout<<"here"<<std::endl;
	if(!initial_pose_set)
	{
		errorSum=0;
		numSample=0;
		initial_pose_set=true;
		pointRp[0]=cur.x;
		pointRp[1]=cur.y;
		pointRp[2]=cur.z;
	}
	pointRc[0]=cur.x-pointRp[0];
	pointRc[1]=cur.y-pointRp[1];
	pointRc[2]=cur.z-pointRp[2];
}
inline float errorUpdate(){
	errorSum+=hypot(hypot(pointT[0]-pointRc[0],pointT[1]-pointRc[1]),pointT[2]-pointRc[2]);
	numSample++;
//	std::cout<<numSample<<std::endl;
	return numSample>1?errorSum/(numSample):1.0f/0.0f;
}
void tstHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::Point cur = odom->pose.pose.position;
	pointT[0]=cur.x;
	pointT[1]=cur.y;
	pointT[2]=cur.z;
	errorUpdate();
}
inline float rand01(){return ((float)rand()/(RAND_MAX));};

int main (int argc, char *argv[])
{
    rosbag::Bag bag;
    std::string filename;

    ros::init(argc, argv, "genetic");
    ros::NodeHandle nh("~");

	std::string testPose,refPose,paramOut,cmdPlay;
	nh.param<std::string>("testPose",testPose,"/laser_odom_to_init");
	nh.param<std::string>("refPose",refPose,"/integrated_to_init");
	nh.param<std::string>("paramOut",paramOut,"/SUREparams");
	nh.param<std::string>("rosbagCommand",cmdPlay
						 ,"rosbag play --clock --quiet ~/withGround/2016-07-14-15-07-23_6.bag");
	float mutProp, crsProp;
	nh.getParam("mutationProp",mutProp);
	nh.getParam("crossoverProp",crsProp);

	nh.getParam("numberOfParameters",numParams);
	nh.getParam("numberOfIterations",numIter);
	float *uLimits=new float(numParams);
	float *lLimits=new float(numParams);
	std::string parName ("parameter");
	char numstr[10];
	for(int i=0;i<numParams;i++)
	{
		sprintf(numstr,"parameter%dL",i);
		nh.getParam(numstr,lLimits[i]);
		sprintf(numstr,"parameter%dU",i);
		nh.getParam(numstr,uLimits[i]);
	}


    ros::Publisher pubFeatParams;
    pubFeatParams = nh.advertise<std_msgs::Float32MultiArray> (paramOut.c_str(), 5);
    ros::Subscriber subPoseTst= nh.subscribe<nav_msgs::Odometry> (testPose.c_str(), 1, tstHandler);
	ros::Subscriber subPoseRef= nh.subscribe<nav_msgs::Odometry> (refPose.c_str(), 1, refHandler);
    float best_score=1.0f/0.0f;
    std::vector<float*> Samples;
    std_msgs::Float32MultiArray msgParams;
    ros::Rate rate(1);

    for(int i=0;i<POP_LEN;i++)
    {
        float *klk= new float[numParams+1];
        for(int j=0;j<numParams;j++)
            klk[j]=((float) rand()) /(RAND_MAX)*(uLimits[j]-lLimits[j])+lLimits[j];
        klk[numParams]=best_score;
        Samples.push_back(klk);
    }
	//std::cout<<numIter<<":"<<numParams<<":"<<paramOut<<":"<<refPose<<":"<<std::endl;
    for(int Nit=0;Nit<numIter;Nit++)
    {
        for (std::vector<float*>::iterator params = Samples.begin()+RET_POP ; params!=Samples.end(); ++params)
        {
            //////////////////////////mutate
            std::memcpy(*params, Samples[rand()%RET_POP],numParams*(sizeof(float)));
			bool isCrossed=(rand01()<crsProp);
            int index=rand()%numParams;
			int whoElse = rand()%RET_POP;
			bool somethingChanged=false;
			do for(int i=0;i<numParams;i++)
			{
				if(isCrossed&&rand01()<0.5&&(somethingChanged=true))
					(*params)[i]=Samples[whoElse][i];
			  	if(rand01()<mutProp&&(somethingChanged=true))
					(*params)[i]=((float) rand()) /(RAND_MAX)*(uLimits[index]-lLimits[index])+lLimits[index];
            }while(!somethingChanged);
            msgParams.data.clear();
            for(int kp=0;kp<numParams;kp++)
                msgParams.data.push_back((*params)[kp]);
            pubFeatParams.publish(msgParams);
            ///////////////////////////o
			initial_pose_set=false;
            pid_t pid=fork();
            if(pid==0)
            {
                setpgid(getpid(),getpid());
                system(cmdPlay.c_str());
                _exit(EXIT_SUCCESS);
            }
            else
            {
                int status;
					sleep(1);
                while(!waitpid(pid,&status,WNOHANG)){
                    ros::spinOnce();
					sleep(1);
                }
				
                (*params)[numParams]=errorUpdate();
                kill(-pid, SIGTERM);
            }
            for(int kp=0;kp<numParams+1;kp++)
                std::cout<<(*params)[kp]<<":";
            std::cout<<std::endl;

      
        }
        std::sort(Samples.begin(),Samples.end(),ConfComp);
        for(int kp=0;kp<numParams+1;kp++)
            std::cout<<Samples[0][kp]<<":";
        std::cout<<"Best   "<<std::endl;
    }
    return 0;
}
