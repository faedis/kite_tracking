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

void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
	key = shutdownkey->data;
}

int main(int argc, char** argv)
{
	// default:
	int fps = 60;
	int frHeight = 360; //576;
	int frWidth = 640; //1024;
/*
	if(argc == 3){
		frWidth = atoi(argv[1]);
		frHeight = atoi(argv[2]);
	}
	else{
		std::cout<< "Default camera values used.\n You can pass the arguments: frame width, frame height in command line\n";
	}	
	std::cout << "\nframe width 	" << frWidth
	<< "\nframe height	" << frHeight << "\n"; 
*/
	// Ros
	ros::init(argc, argv, "camera_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("camera/image_raw", 1);
	ros::Publisher grabbed_pub = nh.advertise<std_msgs::Bool>("grabbed",1);
	ros::Subscriber shutdownkey_sub_  = nh.subscribe("shutdownkey",1,shutdownCb);
	//ros::Publisher waitkey_pub = nh.advertise<std_msgs::Int8>("waitkey",1);
	//ros::Subscriber pixelpos_sub = nh.subscribe("pixelpos",1,pixelposCb);
	sensor_msgs::ImagePtr msg;
	std_msgs::Bool grabbed;
	//std_msgs::Int8 waitkey;
	//geometry_msgs::Pose2D pixelpos;

	// OpenCV
	cv::Mat frame;
  	cv::VideoCapture cap;
	cap.open(0);
	if(!cap.isOpened()){
		std::cout << "Cannot open camera!!!\n";
		return -1;
	}
//	std::string winName = "Preview";
//	cv::namedWindow(winName, cv::WINDOW_KEEPRATIO);

	cap.set(cv::CAP_PROP_FPS,fps);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT,frHeight);
	cap.set(cv::CAP_PROP_FRAME_WIDTH,frWidth);

	bool secondframecounter = false;
	// Loop
	while (nh.ok()) {
		//gettimeofday(&t1, NULL);
		cap.grab();
		secondframecounter ^=true;
		if(secondframecounter){
		grabbed.data = true;
		grabbed_pub.publish(grabbed);
		}
		//gettimeofday(&t2, NULL);
		//elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		//elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		//ROS_INFO("time grab: %f", elapsedTime);
		cap.retrieve(frame);
		//gettimeofday(&t2, NULL);
		//elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		//elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		//ROS_INFO("time retrieved: %f", elapsedTime);
		if(secondframecounter){
			if(!frame.empty()){
				msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
//ROS_INFO("publish:");
				pub.publish(msg);
			}
		}
		ros::spinOnce();
		if(key) {
			ROS_INFO("Shutdown\n");
			ros::shutdown();
			break;
		}	
	}
}
