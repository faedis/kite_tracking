#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
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
	ros::Subscriber pixelpos_sub_;
	ros::Subscriber focus_and_zoom_sub_;
	ros::Publisher comparedpixelpos_pub_;
	
private:
	cv::Mat frame;
	cv::Rect2d roi;
	geometry_msgs::Pose2D targetpixelTracker;
//	cv::Point2f targetpixelTracker;
	geometry_msgs::Pose2D targetpixelAruco;
//	cv::Point2f targetpixelAruco;
	int roisize = 100; // size of region of interest
	int frWidth = 640;
	int frHeight = 360;
	bool centerdetectflag = false;
	geometry_msgs::Pose2D comparedpixelpos;
// tracker:
	cv::Ptr<cv::Tracker> tracker;
	bool initializeflag = true;
	bool trackercreated = false;
	bool trackerdetectflag = false;
	bool firstInitialization = true;
public:
	KCFtracker() : it_(nh_) {
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&KCFtracker::imageCb, this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&KCFtracker::shutdownCb,this);
		// 
		pixelpos_sub_ = nh_.subscribe("pixelpos",1,&KCFtracker::pixelposCb,this);
		// focus_and_zoom_pub_ receives  detectflag, sharpness, target size, center detect flag
		focus_and_zoom_sub_ = nh_.subscribe("fandzmsg",1,&KCFtracker::fandzmsgCb,this);
		//  [0] = detected 0 or 1, [1] = sharpness, [2] = target size, [3] = center detected
		comparedpixelpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("comparedpixelpos",1);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		cv_bridge::CvImageConstPtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv_ptr->image.copyTo(frame);
		if(!initializeflag){
//		ROS_INFO("updateTracker");
			trackerdetectflag = tracker->update(frame,roi);
			if(!trackerdetectflag){
				initializeflag = true;
			ROS_INFO("tracker not detected");
			}
			else{
				targetpixelTracker.x = roi.x + roi.width/2.0;
				targetpixelTracker.y = roi.y + roi.height/2.0;
			}
		}
	}

	void initTracker(){
	//	ROS_INFO("initialize tracker");
		if(!firstInitialization){
		tracker->clear();
		firstInitialization = false;
		}
		cv::Ptr<cv::Tracker> newtracker = cv::Tracker::create("KCF");
		newtracker->init(frame,roi);
		tracker = newtracker;
	}

	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
		if(((pow(targetpixelAruco.x-targetpixelTracker.x,2.0) + pow(targetpixelAruco.y - targetpixelTracker.y,2.0))>900)&&(targetpixelAruco.x>+0)){
//				ROS_INFO("trackerdectectflag %f	%f", targetpixelAruco.y, targetpixelTracker.y);
				trackerdetectflag = false;
				initializeflag = true;			
		}
		targetpixelAruco.x = pixelpos->x;
		targetpixelAruco.y = pixelpos->y;
		targetpixelAruco.theta = pixelpos->theta;
		targetpixelTracker.theta = pixelpos->theta;

	}
	void fandzmsgCb(const std_msgs::Float32MultiArray::ConstPtr& fandzmsg){
		centerdetectflag = fandzmsg->data[3];
//		ROS_INFO("centerdetectflag, initializeflag, trackerdet	%d	%d	%d", centerdetectflag, initializeflag, trackerdetectflag);
		if(!trackerdetectflag && initializeflag && centerdetectflag){
			roi = cv::Rect2d(targetpixelAruco.x - roisize/2.0, targetpixelAruco.y-roisize/2.0, roisize, roisize)&cv::Rect2d(0,0,frWidth-1,frHeight-1);
			initTracker();
			initializeflag = false;
			comparedpixelpos.x = targetpixelAruco.x;
			comparedpixelpos.y = targetpixelAruco.y;
			comparedpixelpos.theta = targetpixelAruco.theta;
			// publish data from aruco detection
			comparedpixelpos_pub_.publish(comparedpixelpos);
		}
		else if(centerdetectflag){
			comparedpixelpos.x = targetpixelAruco.x;
			comparedpixelpos.y = targetpixelAruco.y;
			comparedpixelpos.theta = targetpixelAruco.theta;
			// publish data from aruco detection
			comparedpixelpos_pub_.publish(comparedpixelpos);	
		}
		else if(!centerdetectflag && trackerdetectflag){
			comparedpixelpos.x = targetpixelTracker.x;
			comparedpixelpos.y = targetpixelTracker.y;
			comparedpixelpos.theta = targetpixelAruco.theta;
			// publish data from tracker 
			comparedpixelpos_pub_.publish(comparedpixelpos);
		}
		else{
			comparedpixelpos.x = -1;
			comparedpixelpos.y = -1;
			comparedpixelpos.theta = -1;
			comparedpixelpos_pub_.publish(comparedpixelpos);
		}
		
	}
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown");
		ros::shutdown();
	}

};


int main(int argc, char **argv){

	ros::init(argc, argv, "kcftracker_node");
	KCFtracker kcf;
	ros::spin();
	return 0;

}
