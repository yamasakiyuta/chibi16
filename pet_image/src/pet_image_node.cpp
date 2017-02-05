#include<ros/ros.h>
#include<opencv2/core/core.hpp>
#include<sensor_msgs/image_encodings.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<iostream>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"

//OpenCV
#include <cv.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Labeling.h"

//sensor_msgs::ImagePtr img_msg_out;


using namespace std;
//using namespace cv;
namespace enc = sensor_msgs::image_encodings;

std_msgs::Int32 msg_out;
bool flag_pub = false;


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try{
		//画像の読み込み	
		cv_bridge::CvImage cv_img=*cv_bridge::toCvCopy(msg, enc::BGR8);

		//イメージの宣言	

		cv::Mat rgb = cv_img.image;
		cv::Mat gray;
		cv::Mat smooth;
		cv::Mat img;
		cv::Mat cvT;
		cv::Mat cvAT;

		cv::Mat tmp;
		cv::Mat sobel;

		cv::Mat hsvImage;

		int width = rgb.cols;
		int height = rgb.rows;
		//printf("%d, %d\n", width, height);


		// 平滑化によるノイズ除去
		cv::medianBlur(rgb, smooth, 11);

		// グレースケールへ変換
		//		cv::cvtColor(rgb, gray, CV_BGR2GRAY);	

		// HSV画像へ変換
		cvtColor(smooth, hsvImage, CV_BGR2HSV);

		uchar hue, sat, val;


		cv::Mat red(cv::Size(width,height), CV_8UC1, cv::Scalar(255,0,0) );
		cv::Mat blue(cv::Size(width, height), CV_8UC1, cv::Scalar(255,0,0) );
		//		printf("%d, %d\n", width, height);


		int count = 1;		//0割の防止のためとりあえず1を代入
		int x_sum = 0;		
		for(int y=0; y<height; y++){
			for(int x=0; x<width; x++){

				//H,S,Vの分解
				hue = hsvImage.at<cv::Vec3b>(y,x)[0];
				sat = hsvImage.at<cv::Vec3b>(y,x)[1];
				val = hsvImage.at<cv::Vec3b>(y,x)[2];

				//赤の検出
				if( (hue < 5 || hue >168) && sat > 150 ){
					red.at<uchar>(y,x) = 255;
					x_sum += x;
					count++;
				}else{
					red.at<uchar>(y,x) = 0;
				}	
				/*
				//青の検出
				if( (hue > 97 || hue < 117) && sat > 100 ){
				blue.at<uchar>(y,x) = 0;
				}else{
				blue.at<uchar>(y,x) = 255;
				}
				*/
			}
		}

		if(count > 1500){
		//Labeling
		LabelingBS	labeling;
		RegionInfoBS *ri;

		cv::Mat label(rgb.size(), CV_16SC1);
		labeling.Exec(red.data, (short *)label.data, rgb.cols, rgb.rows, false, 0);	

		
		int ltop_x = 0,
			ltop_y = 0,
			rbottom_x = 0,
			rbottom_y = 0,
			right_x = 0,
			left_x = 0,
			right_y = 0,
			left_y = 0;

		if(0 != labeling.GetNumOfRegions() ){

			for(int i=0; i<10; i++){
				ri = labeling.GetResultRegionInfo(0);
				ri->GetMin(ltop_x,ltop_y);
				ri->GetMax(rbottom_x, rbottom_y);

				int green_x = fabs(ltop_x - rbottom_x);
				int green_y = fabs(ltop_y - rbottom_y);
				if(green_y > green_x){
					left_x = ltop_x;
					left_y = ltop_y;
					right_x = rbottom_x;
					right_y = rbottom_y;
					break;
				}

			}

			rectangle(rgb, cv::Point(left_x, left_y), cv::Point(right_x, right_y), cv::Scalar(0, 255, 255));
		}
		//printf("ltop_x: %d	rbottom_x: %d\n",ltop_x ,rbottom_x);
		int x_value = (ltop_x + rbottom_x) / 2;

		int x_value_percent = x_value * 100 / width;
		msg_out.data = x_value_percent;
		flag_pub = true;

		//printf("x: %d\n",x_value_percent);
//		img_msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb).toImageMsg();


		
		//		cv::Mat line;
		cv::rectangle(sobel, cv::Point(x_value,0), cvPoint(x_value,height), cvScalar(200,0,200), 3, 4);
		}
		else msg_out.data = 0;
		cv::imshow("rgb", rgb);
		//		cv::imshow("smooth", smooth
		//		cv::imshow("cvT",cvT);
		//		cv::imshow("sobel",sobel);
		//		cv::imshow("cvT", cvT);
		//		cv::imshow("cvAT", cvAT);
		//		cv::imshow("red",red);
		//		cv::imshow("sobel",sobel);
		//		cv::imshow("blue",blue);
		//		cv::imshow("label",outimg);		
	}
	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}	
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "pet_image");
	ros::NodeHandle nh;
	ros::NodeHandle h;
//	ros::NodeHandle imgh;
	//to show window
	cv::namedWindow("rgb");
	//	cv::namedWindow("smooth");
	//	cv::namedWindow("red");
	//	cv::namedWindow("label");

	int value = 0;
	int thresholdv = 0; 

	cv::startWindowThread();

	//set Publisher
	ros::Publisher pet_pub = h.advertise<std_msgs::Int32>("/pet_image", 1);
/*
	image_transport::ImageTransport pub(imgh);
        image_transport::Publisher pet_image_pub = pub.advertise("/pet_image_pub", 1);
*/	//for image callback
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);	

	ros::Rate roop_late(10);
	while(ros::ok()){
		//if(flag_pub){
			printf("x:%d\n",msg_out.data);
			pet_pub.publish(msg_out);
//			pet_image_pub.publish(img_msg_out);
			flag_pub = false;
		//}
		ros::spinOnce();
		roop_late.sleep();
	}
	return 0;
}
