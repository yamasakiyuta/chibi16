/////////////////////////////////////////////////////////////////////////////                      
/* chibi16_control
 * 
 * 
 */	
////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
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
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include<sound_play/SoundRequest.h>

nav_msgs::Odometry roomba_odometry_in;
geometry_msgs::Twist cmd_vel;
geometry_msgs::Twist approach_vel;
actionlib_msgs::GoalStatusArray move_base_status;
std_msgs::Int32 pet_pose;
std_msgs::String QR_flag;
std_msgs::Int16 detect_flag;
std_msgs::Int16 arm_comp;

using namespace std;
geometry_msgs::PoseWithCovarianceStamped roomba_amcl_pose;


void approach_vel_Callback(const geometry_msgs::TwistConstPtr& app_msg)
{
	approach_vel=*app_msg;
}

void QR_flagCallback(const std_msgs::StringConstPtr& QR_msg)
{
	QR_flag=*QR_msg;
}


void detect_flagCallback(const std_msgs::Int16ConstPtr& detect_msg)
{
	detect_flag=*detect_msg;
}

void arm_compCallback(const std_msgs::Int16ConstPtr& msg)
{
	arm_comp=*msg;
}

void amcl_poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl_msg)
{
	roomba_amcl_pose=*amcl_msg;
}

void chatterCallback(const std_msgs::Int32ConstPtr& msg)
{
	pet_pose=*msg;
}

void roomba_cmd_Callback(const geometry_msgs::TwistConstPtr& Rcmd_msg)
{
	cmd_vel=*Rcmd_msg;
}

void roomba_status_Callback(const actionlib_msgs::GoalStatusArrayConstPtr& Rsta_msg)
{
	move_base_status=*Rsta_msg;	
}


void 
roomba_odometry_callback(const nav_msgs::OdometryConstPtr& Rodom_msg)
{
	boost::mutex::scoped_lock(roomba_odometry_mutex_);	
	roomba_odometry_in=*Rodom_msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "chibi16_control");
	ros::NodeHandle n_1;
	ros::NodeHandle n_2;
	ros::NodeHandle n_3;
	ros::NodeHandle n;
	ros::NodeHandle n_goal;
	ros::NodeHandle n_amcl;
	ros::NodeHandle nh;
	ros::NodeHandle n_qr;
	ros::NodeHandle n_approach;
	ros::NodeHandle n_detect;
	ros::NodeHandle n_arm;
	ros::NodeHandle n_sound;
        
	ros::Publisher sound_pub = n_sound.advertise<sound_play::SoundRequest>("/robotsound",1);
    
        char f_name[]="/home/amsl/wav/catch.wav";
        sound_play::SoundRequest sound;
        sound.sound=-2;
        sound.command=1;
        sound.volume=3.0;
        sound.arg=f_name;
	 
	ros::Publisher vel_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 100);

	ros::Subscriber roomba_odo_sub = n_1.subscribe("/roomba/odometry", 100, roomba_odometry_callback);

	ros::Subscriber cmd_vel_sub = n_2.subscribe("/cmd_vel", 1,roomba_cmd_Callback);	

	ros::Subscriber approach_vel_sub = n_approach.subscribe("/approach_vel", 1,approach_vel_Callback);	

	ros::Subscriber move_base_status_sub = n_3.subscribe("/move_base/status", 1,roomba_status_Callback);	
	
        ros::Subscriber pet_image_sub = nh.subscribe("/pet_image", 100, chatterCallback);
        
	ros::Subscriber QR_flag_sub = n_qr.subscribe("/qrcode_data", 100, QR_flagCallback);
	
	ros::Subscriber detect_flag_sub = n_detect.subscribe("/start_approach", 1, detect_flagCallback);
       
	ros::Subscriber amcl_pose_sub = n_amcl.subscribe("/amcl_pose", 100, amcl_poseCallback);

	ros::Subscriber arm_comp_sub = n_arm.subscribe("/arm_comp", 100, arm_compCallback);

	ros::Rate loop_rate(10); // ループの周期
	
	double lin = 0.0;
	double ang = 0.0;
	int count = 0;
	int detection = 0;
	int navigation = 1;
	int back_time=35;
	int s_count=0;

	while (ros::ok())
	{	
		if(detect_flag.data==1){
			ROS_INFO("Approaching");
			detection=1;
			navigation=0;
		}
		if(QR_flag.data!="NoData" || arm_comp.data==1){
			if(arm_comp.data==1&&s_count==0){
				sound_pub.publish(sound);
				s_count=1;
			}
		
			ROS_INFO("Exploring");
			detection=0;
			navigation=1;
			detect_flag.data=0;
			QR_flag.data="NoData";
			//arm_comp.data=0;
		}
		if(navigation){
			lin=cmd_vel.linear.x;
			ang=cmd_vel.angular.z;
		}
		else if(detection){
			lin=approach_vel.linear.x;
			ang=approach_vel.angular.z;
		}
		if(arm_comp.data!=0){
			lin=-0.1;
			ang=0;
			count++;
			if(count>back_time){
				count=0;
				arm_comp.data=0;
			}
		}	

	
		//ROS_INFO("I heard: [%d]", roomba_status);
		
		nav_msgs::Odometry roomba_odom;
		roomba_500driver_meiji::RoombaCtrl roomba_target_velocity;
		{
			boost::mutex::scoped_lock(roomba_odometry_mutex_); // mutexロック スコープの範囲だけ有効
			roomba_odom=roomba_odometry_in;
		}
		

//////////////////////////roomba_target_velocityに格納//////////////////////////////////////////////////////////
		roomba_target_velocity.cntl.linear.x = lin;
		roomba_target_velocity.cntl.angular.z = ang;
		roomba_target_velocity.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
		vel_pub.publish(roomba_target_velocity);
		//goal_pub.publish();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			
		ros::spinOnce();
		loop_rate.sleep();
	}

}
