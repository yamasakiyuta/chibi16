#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/Int16.h"
#include <std_srvs/Empty.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std_msgs::Int16 clear_flag;

void clear_flagCallback(const std_msgs::Int16ConstPtr& msg)
{
        clear_flag=*msg;
}

int main(int argc, char** argv){
  	ros::init(argc, argv, "clear_costmap");
	
	ros::NodeHandle n_flag;
	ros::NodeHandle n_clear;

	ros::Subscriber clear_flag_sub = n_flag.subscribe("/clear_flag", 100, clear_flagCallback); 
 
	ros::ServiceClient client = n_clear.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))&&ros::ok()){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	
	int count=0;

	ros::Rate rate(10.0);
	while(ros::ok()){
		std_srvs::Empty srv;
		if(count%10==0)printf("%d\n",11-count/10);
		if(clear_flag.data==1 || count>100){
			if(client.call(srv)){
				ROS_INFO("clear!");
			}
	        	else{
				ROS_ERROR("Faild to call service clear_costmaps");
			}
			count=0;
			clear_flag.data=0;
		}
	
		count++;
		ros::spinOnce();
		rate.sleep();
	}
  	return 0;
}
