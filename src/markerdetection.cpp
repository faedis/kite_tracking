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
/*
Receives frame and processes it
publishes target position and size
*/

struct timeval t1, t2;
double elapsedTime;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// focus measure
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
	float focusMeasure = sigma.val[0] * sigma.val[0];
	return focusMeasure;
}

class ArucoDet{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber shutdownkey_sub_;
	ros::Publisher pixelpos_pub_;
	ros::Publisher targetsize_pub_;
	ros::Publisher detect_pub_;
	ros::Publisher focus_pub_;
	ros::Publisher focus_and_zoom_pub_;
private:
int key = 0;
cv::Mat grayFrame;
cv::Point target;
int frWidth = 640;
int frHeight = 360;
// Messages
geometry_msgs::Pose2D pixelpos;
//std_msgs::Bool detectmsg;
std_msgs::Float32MultiArray fandzmsg;
std::string winName = "ArucoDetection";
bool detectflag = false;
bool knowcenterflag = false;
// Aruco Variables
int dictNumber = 10;
int markerSize = 5;
std::vector< int > markerIds;
std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCandidates;
cv::Point2f v01, v03, v21, v23;
float area1, area2;
//cv::aruco::DetectorParameters parameters;
cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
//cv::aruco::Dictionary dictionary = cv::aruco::generateCustomDictionary(dictNumber,markerSize);
cv::Ptr<cv::aruco::Dictionary> dictionary;
bool centerMark = 0, rightMark = 0, leftMark = 0;
cv::Point2f centerCenter, centerRight, centerLeft;
int centerId = 0, rightId = 0, leftId = 0;
double focusmeasure;

public:
	ArucoDet() : it_(nh_) {
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&ArucoDet::imageCb, this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&ArucoDet::shutdownCb,this);
		// pixelpos_pub_ for ptu cotrol
		pixelpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("pixelpos",1);
		// focus_and_zoom_pub_ sends  detectflag, sharpness, target size
		focus_and_zoom_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("fandzmsg",1);
		fandzmsg.data.resize(4); // [0] = detected 0 or 1, [1] = sharpness, [2] = target size, [3] = center detected

		// Read in parameters:
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
		if(nh_.hasParam("markerdetection/dictNumber")){
			bool success = nh_.getParam("markerdetection/dictNumber",dictNumber);
			ROS_INFO("Read parameter dictNumber, success: %d	%d",dictNumber, success);
		}
		else ROS_INFO("param dictNumber not found");
		if(nh_.hasParam("markerdetection/markerSize")){
			bool success = nh_.getParam("markerdetection/markerSize",markerSize);
			ROS_INFO("Read parameter markerSize, success: %d	%d",markerSize, success);
		}
		else ROS_INFO("param dictNumber not found");

		dictionary = cv::aruco::generateCustomDictionary(dictNumber,markerSize);

	}

	~ArucoDet(){
//		cv::destroyWindow(winName);
	}
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown\n");
		ros::shutdown();
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg){
		gettimeofday(&t1,NULL);
		//ROS_INFO("callback:");
		cv_bridge::CvImageConstPtr cv_ptr;
		try{
			cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
//ROS_INFO("received and read:");

		cv::cvtColor(cv_ptr->image,grayFrame, CV_BGR2GRAY);
		cv::aruco::detectMarkers(grayFrame, dictionary, markerCorners, markerIds);
/*			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;   // us to ms
		ROS_INFO("time of detection: %f", elapsedTime);
*/

		if(markerCorners.size() > 0){
			detectflag = true;
			for(int i = 0; i<markerCorners.size(); i++){
				if(markerIds[i] == 0){
					centerMark = true;
					knowcenterflag = true;
					centerId = i;
					centerCenter = (markerCorners[i][0] + markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3])/4.0;
				}
				if(markerIds[i] == 1){
					rightMark = true;
					rightId = i;
					centerRight = (markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3] + markerCorners[i][0])/4.0;
				}
				if(markerIds[i] == 2){
					leftMark = true;
					leftId = i;
					centerLeft = (markerCorners[i][1] + markerCorners[i][2] + markerCorners[i][3] + markerCorners[i][0])/4.0;
				}
			}
			if(centerMark && (rightMark && leftMark)){
				pixelpos.x = centerCenter.x;
				pixelpos.y = centerCenter.y;
				pixelpos.theta = std::atan2(-(centerLeft.x-centerRight.x),(centerLeft.y-centerRight.y));
			}
			else if(rightMark && leftMark){
				knowcenterflag = true;
				pixelpos.x = (centerLeft.x + centerRight.x)/2.0;
				pixelpos.y = (centerLeft.y + centerRight.y)/2.0;
				pixelpos.theta = std::atan2(-(centerLeft.x-centerRight.x),(centerLeft.y-centerRight.y));
			}
			else if(rightMark && centerMark){
				pixelpos.x = centerCenter.x;
				pixelpos.y = centerCenter.y;
				pixelpos.theta = std::atan2(-(centerCenter.x-centerRight.x),(centerCenter.y-centerRight.y));
			}
			else if(leftMark && centerMark){
				pixelpos.x = centerCenter.x;
				pixelpos.y = centerCenter.y;
				pixelpos.theta = std::atan2(-(centerLeft.x-centerCenter.x),(centerLeft.y-centerCenter.y));
			}
			else if(centerMark){
				pixelpos.x = centerCenter.x;
				pixelpos.y = centerCenter.y;
			}
			else{
				pixelpos.x = -1;
				pixelpos.y = -1;
				pixelpos.theta = -10;
				knowcenterflag = false;
				detectflag = false;
			}
		}
		else{
			knowcenterflag = false;
			detectflag = false;
			pixelpos.x = -1;
			pixelpos.y = -1;
			pixelpos.theta = -10;
		}

/*
		// Delay of 16ms!!!! artificial
//		gettimeofday(&t1,NULL);
		elapsedTime = 0;
		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
		elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;
//		ROS_INFO("time aruco	%f",elapsedTime);
		while(elapsedTime<12.5){
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;   // us to ms
		} */


		pixelpos_pub_.publish(pixelpos);



// Calculate target size
		if(centerMark){
			v01 = markerCorners[centerId][1] - markerCorners[centerId][0];
			v03 = markerCorners[centerId][3] - markerCorners[centerId][0];
			v21 = markerCorners[centerId][1] - markerCorners[centerId][2];
			v23 = markerCorners[centerId][3] - markerCorners[centerId][2];
			area1 = fabs(v01.x*v03.y - v01.y*v03.x);
			area2 = fabs(v21.x*v23.y - v21.y*v23.x);
			fandzmsg.data[2] = sqrt((area1+area2)/2.0);
//			ROS_INFO("area1	%f", area1);
		}
/*
// Draw markers on frame and display
		cv::aruco::drawDetectedMarkers(grayFrame,markerCorners,markerIds);
		cv::imshow(winName,grayFrame);
		cv::waitKey(1);
*/
// clean the vectors containing informationa about the detected markers
		markerCorners.clear();
		markerIds.clear();

		rightMark = false, leftMark = false, centerMark = false;

// these messages are all in focus_and_zoom

		if(knowcenterflag && (fandzmsg.data[2]>3)){
		focusmeasure = LaplaceVarFocus(grayFrame(cv::Rect(round(pixelpos.x-4*fandzmsg.data[2]/2.0),round(pixelpos.y-4*fandzmsg.data[2]/2.0), round(4*fandzmsg.data[2]), round(4*fandzmsg.data[2]))&cv::Rect(0,0,frWidth-1,frHeight-1)));
		}
		else focusmeasure = LaplaceVarFocus(grayFrame);

		fandzmsg.data[0] = detectflag;
		fandzmsg.data[1] = focusmeasure;
		fandzmsg.data[3] = knowcenterflag;

		focus_and_zoom_pub_.publish(fandzmsg);
	}
};




int main(int argc, char **argv)
{

	ros::init(argc, argv, "markerdetection_node");
	ArucoDet ad;
	ros::spin();
	return 0;
}

