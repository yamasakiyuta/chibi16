#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>
#include <move_base/move_base.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/Int16.h"

std_msgs::Int16 QR_flag;	
geometry_msgs::Twist cmd_vel;
std_msgs::Int32 pet_pose;
geometry_msgs::PoseStamped object_pose;
using namespace std;

void QR_flagCallback(const std_msgs::Int16ConstPtr& msg)
{
	QR_flag=*msg;
}

void chatterCallback(const std_msgs::Int32ConstPtr& msg)
{
	pet_pose=*msg;
	cout<<"data: "<<pet_pose.data<<endl;
}

void object_poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	object_pose=*msg;
}

int main(int argc, char** argv){
        ros::init(argc, argv, "object_approach");

	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle n_pet;
	ros::NodeHandle n_QR;
	
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/approach_vel", 100);
	
	ros::Subscriber object_distance = nh.subscribe("/object_pose",1,object_poseCallback);
	
	ros::Subscriber pet_image_sub = n_pet.subscribe("/pet_image", 100, chatterCallback);
	
	//ros::Subscriber pet_image_sub = n_pet.subscribe("/green_image", 100, chatterCallback);
	
	ros::Subscriber QR_flag_sub = n_QR.subscribe("/qr", 100, QR_flagCallback); 
	
	ros::Rate loop_rate(10); // ループの周期

	double lin = 0.0;
	double ang = 0.0;
	int count = 0;
	double last_pet_pose;
	//last_pet_pose=object_direction;
	int pet_direction=1;

	while (ros::ok())
	{	
	//if(QR_flag.data==1){	
		
		if(last_pet_pose<50){
			pet_direction=1;
		}
		else if(last_pet_pose>=50){
			pet_direction=-1;
		}
		
		if(pet_pose.data>0 && pet_pose.data<100){
			lin=0.1-0.1*(50-pet_pose.data)*(50-pet_pose.data)/50/50;
			ang=0.5*pet_direction*(50-pet_pose.data)*(50-pet_pose.data)/50/50;
			if(object_pose.pose.position.x<0.28 && object_pose.pose.position.x>0)lin=0;
		}
		else if(pet_pose.data==0){
			ang=pet_direction*0.4;
			lin=0;
		}
		ROS_INFO("approach_velocity: [%f,%f,%f]",lin ,ang,object_pose.pose.position.x);
		cmd_vel.linear.x=lin;
		cmd_vel.angular.z=ang;
		vel_pub.publish(cmd_vel);		
		ros::spinOnce();
		loop_rate.sleep();
		count++;
		if(pet_pose.data!=0)last_pet_pose=pet_pose.data;
	//}
	//else ROS_INFO("Waiting for flag");
	}
};

