#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Pose2D.h>
#include "opencv2/videoio.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include <math.h>
#include <sys/time.h>
#include <vector>
#include <iostream>




class KCFtracker{

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber shutdownkey_sub_;
	ros::Publisher pixelpos_pub_;
	ros::Publisher focus_and_zoom_pub_;
private:
	cv::Mat roi, frame;
