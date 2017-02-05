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
#include "std_msgs/Int16.h"
//OpenCV
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Labeling.h"



using namespace std;
//using namespace cv;
namespace enc = sensor_msgs::image_encodings;

sensor_msgs::ImagePtr msg_out;

std_msgs::Int32 msg_x_out;
std_msgs::Int16 qr_flag;

//Flags:::::
//labeling flag
bool flag_lab = false;
//publish flag
bool flag_pub = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try	{
		//画像の読み込み	
		cv_bridge::CvImage cv_img=*cv_bridge::toCvCopy(msg, enc::BGR8);

		//イメージの宣言	
		cv::Mat rgb = cv_img.image;
		cv::Mat hsvImage;
		cv::Mat tmp;
		cv::Mat gray;
		cv::Mat dilate_img;
		cv::Mat erode_img;

		int width = rgb.cols;
		int height = rgb.rows;
		//printf("height : %d\n",height);
		//printf("width  : %d\n\n",width);
		int pixel = width * height;
		int count = 0;

		// HSV画像へ変換
		cvtColor(rgb, hsvImage, CV_BGR2HSV);
		uchar hue, sat, val;

		cv::Mat green(cv::Size(width,height), CV_8UC1, cv::Scalar(255,0,0) );
		bool flag_in = false;

		for(int y=0; y<height; y++){
			for(int x=0; x<width; x++){

				//H,S,Vの分解
				hue = hsvImage.at<cv::Vec3b>(y,x)[0];
				sat = hsvImage.at<cv::Vec3b>(y,x)[1];
				val = hsvImage.at<cv::Vec3b>(y,x)[2];

				//緑の検出
				if( (40 < hue && hue < 80 ) && sat > 70 && val > 70){
					green.at<uchar>(y,x) = 255;
					flag_in = true;
					count++;
				}else{
					green.at<uchar>(y,x) = 0;
				}	
			}	
		}

		cvtColor(green,gray,CV_GRAY2RGB);
		
		float pixel_percent;
		pixel_percent = 100 * ( (float)count / pixel );
		printf("count : %d\npixel_percent: %f\n\n",count, pixel_percent); 
		if(pixel_percent>2.0){

			//Labeling
			LabelingBS	labeling;
			RegionInfoBS *ri;
			cv::Mat label(rgb.size(), CV_16SC1);
			bool flag_out = false;
			if(flag_in){
				labeling.Exec(green.data, (short *)label.data, rgb.cols, rgb.rows, false, 0);	
				if(labeling.GetNumOfRegions() > 2) flag_out = true;
			}

			int 	ltop_x = 0,
				ltop_y = 0,
				rbottom_x = 0,
				rbottom_y = 0;

			if(flag_out){
				flag_lab = true;
			}


			if(flag_lab){

				ri = labeling.GetResultRegionInfo( 0 );
				ri->GetMin(ltop_x,ltop_y);
				ri->GetMax(rbottom_x, rbottom_y);

				int labelsize_x =fabs( rbottom_x - ltop_x );
				int labelsize_y =fabs( rbottom_y - ltop_y );

				if(ltop_y < 380) {
					cv::Mat roi_img(gray, cv::Rect(ltop_x, ltop_y, labelsize_x, labelsize_y));

					if(labelsize_x !=0 && labelsize_y !=0){
						cv::Mat qr_code (rgb.cols,rgb.cols,roi_img.type() );
						cv::resize(roi_img, qr_code, qr_code.size(), cv::INTER_CUBIC);
						cv::dilate(qr_code, dilate_img, cv::Mat(), cv::Point(-1,-1), 3);
						cv::erode(dilate_img, erode_img, cv::Mat(), cv::Point(-1,-1), 3);
						rectangle(rgb, cv::Point(ltop_x, ltop_y), cv::Point(rbottom_x, rbottom_y), cv::Scalar(0, 255, 255));

						msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", qr_code).toImageMsg();
						flag_pub = true;
						//imshow("qr_code",erode_img);
						
						int x_value = (ltop_x + rbottom_x) / 2;
						int x_value_percent = x_value * 100 / width;
						msg_x_out.data = x_value_percent;
						
						printf("x: %d\n",x_value_percent);
					}
				}
				flag_lab=false;
			}
		}
		//cv::imshow("green_camera", rgb);
	}

	catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}	
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "green_cam");
	ros::NodeHandle nh;
	ros::NodeHandle h;
	ros::NodeHandle x;

	//to show window	Use Testmode check image!!
	//cv::namedWindow("green_camera");
	//cv::namedWindow("qr_code");
	cv::startWindowThread();

	//set Publisher
	image_transport::ImageTransport pub(h);
	image_transport::Publisher qr_pub = pub.advertise("/qr_image", 1);

	ros::Publisher qr_x_pub = x.advertise<std_msgs::Int32>("/qr_x", 1);
	ros::Publisher qr_flag_pub = x.advertise<std_msgs::Int16>("/Green_flag", 1);
	

	//for image callback
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);	



	ros::Rate roop_late(10);
	while(ros::ok()){
		
		if(flag_pub){
			qr_pub.publish(msg_out);
			//qr_x_pub.publish(msg_x_out);
			qr_flag.data=1;
			flag_pub = false;
		}
		else qr_flag.data=0;

		qr_flag_pub.publish(qr_flag);
		ros::spinOnce();
		roop_late.sleep();
	}



	ros::spin();
	return 0;
}
