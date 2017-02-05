/*
 * Copyright (c) 2013, University of Massachusetts Lowell.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of University of Massachusetts Lowell. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Mikhail Medvedev */

#include <zbar_barcode_reader/barcode_reader.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/String.h>

#include <iostream>

ros::Publisher pub_qr_;
std_msgs::String qrcode; //multi_mapping用pub
std_msgs::String QR_tmp;
class Visualizer
{
	ros::NodeHandle nh_;
	ros::Publisher marker_pub_;
	visualization_msgs::Marker marker_;
	public:
	Visualizer() :
		marker_pub_(nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1))
	{
		marker_.ns = "qrcodes";
		marker_.type = visualization_msgs::Marker::CUBE;
	}

	void publish(float x, float y, float z, const std::string & frame_id, int id)
	{
		geometry_msgs::PoseStamped pose;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = frame_id;
		pose.pose.orientation.w = 1;
		pose.pose.position.x = x;
		pose.pose.position.y = y;
		pose.pose.position.z = z;
		publish(pose, id);
	}
	void publish(const geometry_msgs::PoseStamped & target, int id)
	{
		marker_.id = id;
		marker_.header.stamp = target.header.stamp;
		marker_.header.frame_id = target.header.frame_id;
		marker_.action = visualization_msgs::Marker::ADD;
		marker_.pose = target.pose;

		marker_.scale.x = 0.05;
		marker_.scale.y = 0.05;
		marker_.scale.z = 0.05;
		marker_.color.r = 0.7;
		marker_.color.g = 0.7;
		marker_.color.b = 0.7;
		marker_.color.a = 0.1;
		marker_.lifetime = ros::Duration(0);
		marker_pub_.publish(marker_);
	}
};

/**
 * Publish objects to /tf
 */

int publish_no = 0;   //added

class ObjectTfPub
{
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		ros::Publisher pub_tf_;
		ros::Publisher pub_tf_private_;
	public:
		ObjectTfPub() :
			private_nh_("~"),
			pub_tf_(nh_.advertise<tf::tfMessage>("/tf", 5)),
			pub_tf_private_(private_nh_.advertise<tf::tfMessage>("tf_objects", 5))
	{
	}
		void publish(std::vector<barcode::Barcode> & barcodes, const sensor_msgs::ImageConstPtr& msg) const
		{	
			
			tf::tfMessage tf_msg;
			geometry_msgs::TransformStamped tr;
			tr.header = msg->header;
			//if( barcodes.size() == 0 ) return;
			qrcode.data = "NoData"; //multi_mapping用pub  changed
			for (uint i = 0; i < barcodes.size(); i++)
			{
				barcode::Barcode & barcode = barcodes[i];
				tr.child_frame_id = barcode.data;
				tr.transform.rotation.w = 1;
				tr.transform.translation.x = barcode.x;
				tr.transform.translation.y = barcode.y;
				tr.transform.translation.z = barcode.z;
				tf_msg.transforms.push_back(tr);
				qrcode.data = barcode.data; //multi_mapping用pub
				
				if(qrcode.data == QR_tmp.data){
					qrcode.data = "NoData";
				}
			}
			if(qrcode.data != "NoData"){
				pub_qr_.publish(qrcode); //multi_mapping用pub
				std::cout<<std::setw(5)<<publish_no<<" :  "<<qrcode.data<<std::endl;   //aded
			}
			/* the first signal publish only	*/
			QR_tmp.data = qrcode.data;
			publish_no++;   //added
			pub_tf_.publish(tf_msg);
			pub_tf_private_.publish(tf_msg);
		}
};

class Node
{
	private:
		barcode::BarcodeReader reader_;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber sub_image_;

		Visualizer vis_;
		ObjectTfPub object_pub_;

		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			cv_bridge::CvImagePtr cv_img_ptr;
			msg->header;

			try
			{
				cv_img_ptr = cv_bridge::toCvCopy(msg, "mono8");
			}
			catch (cv_bridge::Exception& e)
			{
				ROS_ERROR_STREAM("Could not convert ROS image to CV: "<< e.what());
				return;
			}

			static int id = 0;
			int n = reader_.parse(cv_img_ptr);
			ROS_DEBUG_STREAM("Got image, has "<<n<<" barcodes");
			std::vector<barcode::Barcode> barcodes = reader_.getBarcodes();
			for (uint i = 0; i < reader_.getBarcodes().size(); i++)
			{
				ROS_DEBUG_STREAM("Barcode: " << barcodes[i].data //
						<< " x:"<<barcodes[i].x//
						<< " y:"<<barcodes[i].y);
				vis_.publish(barcodes[i].x, barcodes[i].y, barcodes[i].z, msg->header.frame_id, id++ % 1000);
			}
			if (msg->header.frame_id == "")
			{
				//ROS_ERROR_THROTTLE(1, "Received image with empty frame_id, would cause tf connectivity issues.");
			}
			object_pub_.publish(barcodes, msg);
		}

	public:
		Node() :
			it_(nh_),
			sub_image_(it_.subscribe("qr_image", 1, &Node::imageCallback, this))
	{
		reader_.setBarcodeSize(0.16).setFOV(60);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "qrcode_reader");
	ros::NodeHandle n;
	pub_qr_ = n.advertise<std_msgs::String>("qrcode_data", 1);
	Node node;
	ros::spin();
	return 0;
}

