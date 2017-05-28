//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\
// Purpose:
// 1) Notice the PTU serial node of the arrival of every fourth frame that has arrived
// 2) Retreive every fourth frame and publish it
//
//
// Author: Guetg Fadri, guetgf@student.ethz.ch
// 28.05.2017
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <sys/time.h>
#include <string>
#include <stdlib.h>
#include <iostream>

bool key = false;

// Shutdown Callback
void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
	key = shutdownkey->data;
}

int main(int argc, char** argv)
{
	// camera settings:
	int fps = 60;
	int frHeight = 360; 
	int frWidth = 640; 
	int cameraNumber = 0; 
	// counter for publish the grabbed message when the fourth frame has arrived:
	int fourthframecounter = 0;

	ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh;
	// Initialize publishers and subscribers:
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
	ros::Publisher grabbed_pub = nh.advertise<std_msgs::Bool>("grabbed",1);
	ros::Subscriber shutdownkey_sub_  = nh.subscribe("shutdownkey",1,shutdownCb);
	sensor_msgs::ImagePtr msg;		// message with which the frame will be sent
	std_msgs::Bool grabbed;			// message when new frame has been grabbed

	// Read in parameters that are defined in launch file:
	if(nh.hasParam("camera/frWidth")){
		bool success = nh.getParam("camera/frWidth",frWidth);
		ROS_INFO("Read parameter frWidth, success: %d	%d",frWidth, success);
	}
	else ROS_INFO("param width not found");
	if(nh.hasParam("camera/frHeight")){
		bool success = nh.getParam("camera/frHeight",frHeight);
		ROS_INFO("Read parameter frHeight, success: %d	%d",frHeight, success);
	}
	else ROS_INFO("param height not found");
	if(nh.hasParam("camera/cameraNumber")){
		bool success = nh.getParam("camera/cameraNumber",cameraNumber);
		ROS_INFO("Read parameter cameraNumber, success: %d	%d",cameraNumber, success);
	}
	else ROS_INFO("param cameraNumber not found");
	
	// OpenCV
	cv::Mat frame;
  	cv::VideoCapture cap;
	cap.open(cameraNumber);
	if(!cap.isOpened()){ // will fail if wrong camera number
		std::cout << "Cannot open camera!!!\n";
		return -1;
	}

	// Set the grabbing parameters:
	cap.set(cv::CAP_PROP_FPS,fps);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT,frHeight);
	cap.set(cv::CAP_PROP_FRAME_WIDTH,frWidth);

	// Loop:
	while (nh.ok()) {
		cap.grab();
		if(fourthframecounter>3){
		grabbed.data = true;
		// publish the arrivel of every fourth frame:
		grabbed_pub.publish(grabbed);
		}
		if(fourthframecounter>3){
			cap.retrieve(frame);
			if(!frame.empty()){
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
				// publish the frame:
				pub.publish(msg);
			}
			fourthframecounter = 0;
		}
		// update fraecounter:
		fourthframecounter++;
		// receive single callback for shutdown message:
		ros::spinOnce();
		if(key) {
			ROS_INFO("Shutdown\n");
			ros::shutdown();
			break;
		}	
	} // END loop
}

