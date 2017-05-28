//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\
// Purpose:
// 1) Detect Aruco marker
// 2) Get the size of the marker
// 3) Calculate the sharpness measure
//
//
// Author: Guetg Fadri, guetgf@student.ethz.ch
// 28.05.2017
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <opencv2/aruco.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <vector>
#include <iostream>

// Time variables:
struct timeval t1, t2;
double elapsedTime;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Function for sharpness measure
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
float LaplaceVarFocus(cv::Mat inputFrame) {
	if (inputFrame.empty()) {
		std::cout << "Invalid input for Focus calculation\n";
		return -1;
	}

	int ddepth = CV_32F;
	int kernel_size = 1;
	int scale = 1;
	int delta = 0;
	cv::Mat lap;
	cv::Mat viz;
	// GaussianBlur(inputFrame,inputFrame, Size(3,3), 0, 0, BORDER_DEFAULT );
	cv::Laplacian(inputFrame, lap, ddepth, kernel_size, scale, delta, cv::BORDER_DEFAULT);
	cv::convertScaleAbs(lap, viz);
	cv::Scalar mu, sigma;
	cv::meanStdDev(lap, mu, sigma);
	float sharpnessMeasure = sigma.val[0] * sigma.val[0];
	return sharpnessMeasure;
}
// END Function for sharpness measure
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Aruco Marker Detection Class
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ArucoDet{
	ros::NodeHandle nh_;
	// The ROS subscribers and publishers:
	image_transport::ImageTransport it_;		// for receiving the frame
	image_transport::Subscriber image_sub_;		// for receiving the frame 
	ros::Subscriber shutdownkey_sub_;
	ros::Publisher pixelpos_pub_;
	ros::Publisher focus_and_zoom_pub_;
private:
int key = 0;
cv::Mat grayFrame;	
int frWidth = 640;		// frame width
int frHeight = 360;		// frame height
double sharpnessmeasure;	// sharpness measure
double artDelay = 29.0; // artificial delay for timing the sending of the pixelposition
// Messages:
geometry_msgs::Pose2D pixelpos;
std_msgs::Float32MultiArray fandzmsg;

bool detectflag = false;
// Aruco Variables:
// see http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html for more information
int dictNumber = 1; 
int markerSize = 3;
std::vector< int > markerIds;
std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
cv::Ptr<cv::aruco::Dictionary> dictionary;
//
cv::Point2f v01, v03, v21, v23;	// points for calculating areas
float area1 = 100, area2 = 100; // areas used for calculating size of marker
cv::Point2f targetCenter;		// center of detected object
double theta = -10;				// heading angle of the marker/kite

public:
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Constructor
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	ArucoDet() : it_(nh_) {
		// initialize subscriber and putlisher:
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&ArucoDet::imageCb, this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&ArucoDet::shutdownCb,this);
		pixelpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pixelpos",1);
		// focus_and_zoom_pub_ sends  detectflag, sharpness, target size
		focus_and_zoom_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("fandzmsg",1);
		fandzmsg.data.resize(3); // [0] = detected 0 or 1, [1] = sharpness, [2] = target size

		// Read in parameters that are defined in launch file:
		if(nh_.hasParam("camera/frWidth")){
			bool success = nh_.getParam("camera/frWidth",frWidth);
			ROS_INFO("Read display parameter frWidth, success: %d	%d",frWidth, success);
		}
		else ROS_INFO("display param not found");
		if(nh_.hasParam("camera/frHeight")){
			bool success = nh_.getParam("camera/frHeight",frHeight);
			ROS_INFO("Read parameter frHeight, success: %d	%d",frHeight, success);
		}
		else ROS_INFO("param height not found");
		if(nh_.hasParam("singlemarkerdetection/dictNumber")){
			bool success = nh_.getParam("singlemarkerdetection/dictNumber",dictNumber);
			ROS_INFO("Read parameter dictNumber, success: %d	%d",dictNumber, success);
		}
		else ROS_INFO("param dictNumber not found");
		if(nh_.hasParam("singlemarkerdetection/markerSize")){
			bool success = nh_.getParam("singlemarkerdetection/markerSize",markerSize);
			ROS_INFO("Read parameter markerSize, success: %d	%d",markerSize, success);
		}
		else ROS_INFO("param dictNumber not found");
		if(nh_.hasParam("singlemarkerdetection/artDelay")){
			bool success = nh_.getParam("singlemarkerdetection/artDelay",artDelay);
			ROS_INFO("Read parameter artDelay, success: %f	%d",artDelay, success);
		}
		else ROS_INFO("param artDelay not found");
		// assign aruco detection parameters (http://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html)
		dictionary = cv::aruco::generateCustomDictionary(dictNumber,markerSize);
		parameters->adaptiveThreshWinSizeMax = 23.0;	//default
		parameters->adaptiveThreshWinSizeMin = 3.0;		// default	
		parameters->adaptiveThreshWinSizeStep = 20.0;	// default = 10
		parameters->adaptiveThreshConstant = 13.0;		// default = 7
		parameters->minMarkerPerimeterRate = 0.04;		// defualt = 0.03
		parameters->maxMarkerPerimeterRate = 0.4;		// default = 4
	}
	// END Constructor
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Destructor:
	~ArucoDet(){
	}
	// Shut down the node:
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown\n");
		ros::shutdown();
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Image callback
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		// Start time measurement:
		gettimeofday(&t1,NULL);
		// Receive image:
		cv_bridge::CvImageConstPtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		// Convert image to grayscale:
		cv::cvtColor(cv_ptr->image,grayFrame, CV_BGR2GRAY);
		// Apply the marker detection:
		cv::aruco::detectMarkers(grayFrame, dictionary, markerCorners, markerIds);

		// Read info of detected markers:
		if(!markerCorners.empty()){
			if (markerIds[0] == 0){ // We used ID 0, otherwise adjust
				detectflag = true;
				// get center of marker:
				targetCenter = (markerCorners[0][0] + markerCorners[0][1] + markerCorners[0][2] + markerCorners[0][3])/4.0;
				pixelpos.x = targetCenter.x;
				pixelpos.y = targetCenter.y;
				// Calculate the heading angle:
				// INFO: we use only edges of marker which are in line with angle of attack.
				// the other two edges are more prone to be bended because of the kite structure
				// if you want to use all edges, uncomment the two lines below and adjust other three lines below accordingly
				//theta = std::atan2(-(markerCorners[0][0].x-markerCorners[0][1].x),(markerCorners[0][0].y-markerCorners[0][1].y));
				//theta += std::atan2(-(markerCorners[0][3].x-markerCorners[0][2].x),(markerCorners[0][3].y-markerCorners[0][2].y));
				theta = std::atan2(-(markerCorners[0][1].y-markerCorners[0][2].y),(markerCorners[0][1].x-markerCorners[0][2].x));
				theta += std::atan2(-(markerCorners[0][0].y-markerCorners[0][3].y),(markerCorners[0][0].x-markerCorners[0][3].x));
				theta /= 2.0; // average
				pixelpos.theta = theta;
			}
			else{	// if not the right marker detected:
				pixelpos.x = -1;
				pixelpos.y = -1;
				pixelpos.theta = -10;
				detectflag = false;
			}

		}
		else{	// if nothing detected:
			pixelpos.x = -1;
			pixelpos.y = -1;
			pixelpos.theta = -10;
			detectflag = false;
		}
		// Delay the sending of the pixel position message:
		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
		elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;   // us to ms
		while(elapsedTime<artDelay){
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;   // us to ms
		}

		// Send the position and angle of the marker
		pixelpos_pub_.publish(pixelpos);

		// Calculate marker size:
		if(detectflag){
			v01 = markerCorners[0][1] - markerCorners[0][0];
			v03 = markerCorners[0][3] - markerCorners[0][0];
			v21 = markerCorners[0][1] - markerCorners[0][2];
			v23 = markerCorners[0][3] - markerCorners[0][2];
			area1 = fabs(v01.x*v03.y - v01.y*v03.x);
			area2 = fabs(v21.x*v23.y - v21.y*v23.x);
			// Assign the marker size to the message:
			fandzmsg.data[2] = sqrt((area1+area2)/2.0);
		}
		// clean the vectors containing informationa about the detected markers
		markerCorners.clear();
		markerIds.clear();

		// Apply the focus measure algorithm on the detected object:
		if(detectflag && (fandzmsg.data[2]>3)){
		sharpnessmeasure = LaplaceVarFocus(grayFrame(cv::Rect(round(pixelpos.x-6*fandzmsg.data[2]/2.0),round(pixelpos.y-6*fandzmsg.data[2]/2.0), round(6*fandzmsg.data[2]), round(6*fandzmsg.data[2]))&cv::Rect(0,0,frWidth-1,frHeight-1)));
		}
		else sharpnessmeasure = LaplaceVarFocus(grayFrame); // If not detected on whole frame
		// Assign the detectflag and the sharpness measure to the message:
		fandzmsg.data[0] = detectflag;
		fandzmsg.data[1] = sharpnessmeasure;

		focus_and_zoom_pub_.publish(fandzmsg);
	}
	// END Image callback
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
};

// END Aruco Marker Detection Class
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Main function
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{

	ros::init(argc, argv, "singlemarkerdetection_node");
	// construct object of Aruco Marker Detection Class:	
	ArucoDet ad;
	// Constantly call the callbacks:
	ros::spin();
	return 0;
}
// END Main function
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
