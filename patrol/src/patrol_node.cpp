#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/Int16.h"
#include <sound_play/SoundRequest.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::PoseWithCovarianceStamped init_pose;
geometry_msgs::PoseWithCovarianceStamped roomba_amcl_pose;
std_msgs::Int16 QR_flag;

void init_pose_Callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
         init_pose=*msg;
}

void amcl_poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
        roomba_amcl_pose=*msg;
}

void QR_flagCallback(const std_msgs::Int16ConstPtr& msg)
{
        QR_flag=*msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "patrol");
  int count=0;
  
  ros::NodeHandle nh;
  ros::NodeHandle n;
  ros::NodeHandle n_amcl;
  ros::NodeHandle n_qr;
  ros::NodeHandle n_servo;
  ros::NodeHandle n_goal;
  ros::NodeHandle n_sound;

        ros::Publisher sound_pub = n_sound.advertise<sound_play::SoundRequest>("/robotsound",1);

  ros::Subscriber initialpose_sub = nh.subscribe("/initialpose",1,init_pose_Callback);
  
  ros::Subscriber roomba_amcl_pose_sub = n_amcl.subscribe("/amcl_pose",100,amcl_poseCallback);

  ros::Subscriber QR_flag_sub = n_qr.subscribe("/arm_comp", 100, QR_flagCallback);
 
  ros::Publisher initialpose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
  
  ros::Publisher servo_pub = n_servo.advertise<std_msgs::Int16>("/servo",1);
  
  ros::Publisher goal_pub = n_goal.advertise<std_msgs::Int16>("/first_lap_goal",1);

 //////////////////////sound_play////////////////////////        
        char start[]="/home/amsl/wav/start.wav";
        char fanfale[]="/home/amsl/wav/goal.wav";
        sound_play::SoundRequest sound;
        sound.sound=-2;
        sound.command=1;
        sound.volume=1.0;
        sound.arg=start;

        char say_text[]="QR has been saved";
        sound_play::SoundRequest say;
        say.sound=-3;
        say.command=1;
        say.volume=15.0;
        say.arg=say_text;
        say.arg2="voice_kal_diphone";
 
  
  geometry_msgs::PoseWithCovarianceStamped roomba_initialpose;
  std_msgs::Int16 first_lap_goal;
  first_lap_goal.data=1;
  
  roomba_initialpose.header.frame_id = "map";
  roomba_initialpose.header.stamp = ros::Time::now();
  

  geometry_msgs::PoseWithCovarianceStamped waypoint[10];
  std_msgs::Int16 servo;
  servo.data=2;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))&&ros::ok()){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  sound_pub.publish(sound);
  roomba_initialpose=roomba_amcl_pose;
  roomba_initialpose.pose.pose.position.x=0.0;
  roomba_initialpose.pose.pose.position.y=0.0;
  roomba_initialpose.pose.pose.orientation.z=0.0;
  roomba_initialpose.pose.pose.orientation.w=1.0;
  roomba_initialpose.pose.covariance={0.25,0.0,0.0,0.0,0.0,0.0,0.0,0.25,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.06853891945200942}; 
  initialpose_pub.publish(roomba_initialpose);

  int num=10;

  move_base_msgs::MoveBaseGoal goal;
  move_base_msgs::MoveBaseGoal patrol[10];

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
 
/*  patrol[0].target_pose.pose.position.x = 3.3;
  patrol[0].target_pose.pose.position.y = 0.0;
  patrol[0].target_pose.pose.orientation.w = 1.0;

  patrol[1].target_pose.pose.position.x = 2.85;
  patrol[1].target_pose.pose.position.y = 0.92;
  patrol[1].target_pose.pose.orientation.w = 1.0;

  patrol[2].target_pose.pose.position.x = 4.1;
  patrol[2].target_pose.pose.position.y = 2.0;
  patrol[2].target_pose.pose.orientation.w = 1.0;

  patrol[3].target_pose.pose.position.x = 3.26;
  patrol[3].target_pose.pose.position.y = 1.28;
  patrol[3].target_pose.pose.orientation.w = 1.0;

  patrol[4].target_pose.pose.position.x = 2.05;
  patrol[4].target_pose.pose.position.y = 2.4;
  patrol[4].target_pose.pose.orientation.w = 1.0;

  patrol[5].target_pose.pose.position.x = 1.37;
  patrol[5].target_pose.pose.position.y = 1.67;
  patrol[5].target_pose.pose.orientation.w = 1.0;

  patrol[6].target_pose.pose.position.x = 0.4;
  patrol[6].target_pose.pose.position.y = 2.1;
  patrol[6].target_pose.pose.orientation.w = 1.0;

  patrol[7].target_pose.pose.position.x = 0.0;
  patrol[7].target_pose.pose.position.y = 0.0;
  patrol[7].target_pose.pose.orientation.w = 1.0;
*/
  patrol[0].target_pose.pose.position.x = 0.96;
  patrol[0].target_pose.pose.position.y = 0.809;
  patrol[0].target_pose.pose.orientation.w = 1.0;

  patrol[1].target_pose.pose.position.x = 0.024;
  patrol[1].target_pose.pose.position.y = 1.86;
  patrol[1].target_pose.pose.orientation.w = 1.0;

  patrol[2].target_pose.pose.position.x = 1.7;
  patrol[2].target_pose.pose.position.y = 2.61;
  patrol[2].target_pose.pose.orientation.w = 1.0;

  patrol[3].target_pose.pose.position.x = 1.81;
  patrol[3].target_pose.pose.position.y = 1.13;
  patrol[3].target_pose.pose.orientation.w = 1.0;

  patrol[4].target_pose.pose.position.x = -0.153;
  patrol[4].target_pose.pose.position.y = -0.041;
  patrol[4].target_pose.pose.orientation.w = 1.0;


  goal.target_pose.pose=patrol[0].target_pose.pose;
  
ros::Rate rate(10.0);
while(ros::ok()){
  ROS_INFO("Sending goal %d",count);
  ac.sendGoal(goal);
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
	ROS_INFO("Succeeded goal %d",count);
	waypoint[count]=roomba_amcl_pose;
	//servo_pub.publish(servo);
	if(count==4){
		sound.arg=fanfale;
		sound_pub.publish(sound);
		goal_pub.publish(first_lap_goal);
		break;
	}
	count++;
	goal.target_pose.pose=patrol[count].target_pose.pose;
  }
  else  ROS_INFO("The base failed to move");
  ros::spinOnce();
  rate.sleep();
  
  }
  return 0;
}
