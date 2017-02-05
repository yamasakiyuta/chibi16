/*
just written with establish and publish


*/
////////////////////////////////////////////

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <stdlib.h>


////////////////include from QRcode///////////////////////
#include <sstream>
#include <std_msgs/String.h>
///////////////////////////////////////////////////////////////////

#define N 50;

typedef float Matrix [N][3];


int amount;//	to include from Qrcode, establish world variable



//////////the callback function to include value//////////
void 
roomba_odometry_callback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(roomba_odometry_mutex_);//locking something (i have to look for it)	
	roomba_odometry_in=*msg;
}

//////////////////////////////////////////////////////////////////////////////


///////////////callback function toincllude value///////////////////////

void
String_callback(const std_msgs::String::ConstPtr& msg)
{
		
	amount = atoi(msg->data.c_str());
	
	printf("amount=%lf\n",amount);	
}

////////////////////////////////////////////////////////////////////////////

//////////////////exchange the row with the number from QRcode/////////////////////
void replace(float Matrix A,int full){
	float min = A[0][0];
	int a=0;
	for(int j=1;j<full;j++){
		for(int l=0;l<full;l++){
			if(min<A[j][0]){
				for(k=0;k<3;k++){
					float temp = A[a][k];
					A[a][k]=A[j][k];
					A[j][k]=temp;
				}
			min = A[j][0];
			a = j;
			}
		}
	}
}
/////////////////////////////////////////////////////////////////////
			

int main(int argc,char** argv)
{
	Matrix A;
	int i=0;
	ros::init(argc, argv, "chibi14_motion_sample");
	ros::NodeHandle n;
	
	ros::Publisher vel_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 100);
	
	ros::Subscriber roomba_odo_sub = n.subscribe("/roomba/odometry", 100, roomba_odometry_callback);
	
///////////////////subscribing from QRcode////////////////////////
	ros::Subscriber String_sub	= n.subscribe("/qrcode_data",1,String_callback);
//////////////////////////////////////////
	ros::Rate loop_rate(10); // cycle of roop
	
	while(ros::ok())//repeat
	{	
		nav_msgs::Odometry roomba_odom;								
		roomba_500driver_meiji::RoombaCtrl roomba_target_velocity;
		{
			boost::mutex::scoped_lock(roomba_odometry_mutex_); // mutexロック スコープの範囲だけ有効
			roomba_odom=roomba_odometry_in;
		}															//値を取り込むときに必要みたい
		
		if(amount!=100){//put the value from QRcode in the Matrix when its not a 100
			A[i][0] = amount;
			A[i][1] = roomba_odom.pose.pose.position.x;
			A[i][2] = roomba_odom.pose.pose.position.y;
			
			i++;
		}
		
		
		
		
		
		
		
		
		
		
//////////////////////////roomba_target_velocityに格納//////////////////////////////////////////////////////////
		roomba_target_velocity.cntl.linear.x = lin;
		roomba_target_velocity.cntl.angular.z = ang;
		roomba_target_velocity.mode = roomba_500driver_meiji::RoombaCtrl::DRIVE_DIRECT;
		vel_pub.publish(roomba_target_velocity);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		
		
		
		
	ros::spinOnce();
	loop_rate.sleep();
	}
	
	replace(A,i);

}

