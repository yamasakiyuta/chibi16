#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"

sensor_msgs::LaserScan scan;
std_msgs::Int16 QR_flag;
std_msgs::Int16 detect_flag;

bool laser_flag=false;

void scanCallback(const sensor_msgs::LaserScan &msg)
{
        scan=msg;
	laser_flag=true;
}

void QR_flagCallback(const std_msgs::Int16ConstPtr& msg)
{
        QR_flag=*msg;
}

void detect_flagCallback(const std_msgs::Int16ConstPtr& msg)
{
        detect_flag=*msg;
}


int main(int argc, char** argv){
	ros::init(argc, argv, "object_distance");
	ros::NodeHandle n_scan;
	ros::NodeHandle n;
	ros::NodeHandle n_arm;
	ros::NodeHandle n_qr;
        ros::NodeHandle n_detect;
        ros::NodeHandle n_dist;
	
	tf::TransformBroadcaster br;
	tf::Transform transform;

	//tf::TransformListener listener;
	//tf::StampedTransform transform_1;


	ros::Subscriber scan_sub = n_scan.subscribe("/scan", 100, scanCallback);
	ros::Publisher object_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/object_pose", 100);

	ros::Publisher arm_pub = n.advertise<std_msgs::Int16>("/servo", 1);
	std_msgs::Int16 arm_flag;
	
	ros::Publisher distance_pub = n_dist.advertise<std_msgs::Float32>("/distance", 1);
	std_msgs::Float32 distance;

	ros::Subscriber QR_flag_sub = n_qr.subscribe("/arm_comp", 100, QR_flagCallback);

        ros::Subscriber detect_flag_sub = n_detect.subscribe("/start_approach", 1, detect_flagCallback);

	
	int count_ok=0;
	int detection=0;
	int navigation=1;
	
	geometry_msgs::PoseStamped object_pose;
	object_pose.header.frame_id="object_frame";

	ros::Time current_time, last_time;
        current_time = ros::Time::now();
        last_time = ros::Time::now();

	ros::Rate rate(10.0);

	while (ros::ok()){
		double count_distant=0;
		double count_close=0;
		double sum=0;
		double ave=0;
		double data_num=0;
		double min=20;//scan.ranges[240];

		//object_pose.header.stamp = ros::Time::now();
		
		/*try{
			listener.lookupTransform("/object_frame", "/base_frame",
                        ros::Time(0), transform_1);
    		}	
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}*/	
		if(detect_flag.data==1){
			ROS_INFO("detect");
                        detection=1;
                        navigation=0;
                }
                if(QR_flag.data==1){
			ROS_INFO("navi");
                        detection=0;
                        navigation=1;
			detect_flag.data=0;
                }


		if(detection){
			ROS_INFO("distance: [%f][%f][%f]", scan.ranges[240],scan.ranges[250],scan.ranges[260]);
			for(int i=240;i<=260;i++){
				if((scan.ranges[i]<10 && scan.ranges[i]>0) ){
					if(scan.ranges[i]>0.5)count_distant++;
					if(scan.ranges[i]<0.4)count_close++;
					sum+=scan.ranges[i];
					data_num++;
				}
				if(scan.ranges[i]<min){
					min=scan.ranges[i];
				}
			}
			ave=sum/data_num;
		//	object_pose.pose.position.x=ave;

			ROS_INFO("average of distance [%f]",ave);
			ROS_INFO("min of distance [%f]",min);
			if(count_distant/data_num > 0.8){
				arm_flag.data=0;
				ROS_INFO("DISTANT[%d]",arm_flag.data);
				count_ok-=3;
			}
			else if(count_close/data_num > 0.8){
				arm_flag.data=0;
				ROS_INFO("CLOSE[%d]",arm_flag.data);
				count_ok-=3;
			}
			/*else{
				ROS_INFO("OK");
				count_ok+=2;
			}*/
			//if(count_ok > 5){
			if(min<0.28){
				arm_flag.data=1;
				ROS_INFO("ARM is READY[%d]",arm_flag.data);
			}
			if(count_ok > 30){
				count_ok=30;
			}
			else if(count_ok < -10){
				count_ok=-10;
			}
			arm_pub.publish(arm_flag);
			arm_flag.data=0;
		}

		transform.setOrigin( tf::Vector3(min, 0.0, 0.0) );
		transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_frame", "object_frame"));

		object_pose.pose.position.x=min;
		object_pose_pub.publish(object_pose);

		distance.data=min;
		distance_pub.publish(distance);

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
};
