//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\
// Purpose:
// 1) Display the frame and the detected object
// 2) Interface between the user and the control
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
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>
#include <string>
#define _USE_MATH_DEFINES
#include <math.h>

// CV Window:
std::string winNamedet = "Preview Detection";

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Display Image Class
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ImageShow{
	ros::NodeHandle nh_;
	// The ROS subscribers and publishers:
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber pixelpos_sub_;
	ros::Subscriber comparedpixelpos_sub_;
	ros::Subscriber target_size_sub_;
	ros::Subscriber focus_and_zoom_sub_;
	ros::Publisher waitkey_pub_;
	ros::Publisher shutdownkey_pub_;
private:
cv::Mat frame;
double tsize;				// size of the marker
double theta = 0;			// heading of the marker
double pixelX, pixelY;		// Center of the marker in pixel
// Messages:
std_msgs::Int8 waitkey;
std_msgs::Bool shutdownkey;
//
int frWidth = 640;			// frame width in pixel
int frHeight = 360;			// frame hight int pixel
// Different colors for displaying:
cv::Scalar ArucoCol = cv::Scalar(0,0,255);
cv::Scalar AngleCol = cv::Scalar(255,0,0);
public:
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//	Constructor
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	ImageShow() : it_(nh_) {
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&ImageShow::imageCb, this);
		pixelpos_sub_ = nh_.subscribe("pixelpos",1, &ImageShow::pixelposCb,this);
		focus_and_zoom_sub_ = nh_.subscribe("fandzmsg",1,&ImageShow::displayDetCb, this);
		waitkey_pub_ = nh_.advertise<std_msgs::Int8>("waitkey",1);
		shutdownkey_pub_ = nh_.advertise<std_msgs::Bool>("shutdownkey",1);

		// Read parameters that are defined in launch file
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


		cv::namedWindow(winNamedet,cv::WINDOW_KEEPRATIO);
	}
	//	END Constructor
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	// Destructor:
	~ImageShow(){
		cv::destroyWindow(winNamedet);
	}
	
	// Receive the image:
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
	}
	// Receive the pixel position and heading angle of the marker
	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
		pixelX = pixelpos->x;
		pixelY = pixelpos->y;
		theta = pixelpos->theta;
	}
	// Display the frame with the detected marker and heading angle
	void displayDetCb(const std_msgs::Float32MultiArray::ConstPtr& fandzmsg){
		if(!frame.empty()){
			tsize = fandzmsg->data[2];
			if(pixelX>=0){ // if detected...
			cv::rectangle(frame, cv::Rect2d(pixelX-tsize/2.0,pixelY-tsize/2.0, tsize, tsize)&cv::Rect2d(0,0,frWidth-1,frHeight-1),
				ArucoCol,2,8,0);
			if(theta>-7){ // if detected ... 
				cv::arrowedLine(frame, cv::Point2f(pixelX,pixelY), cv::Point2f(pixelX + std::cos(theta)*30,pixelY - std::sin(theta)*30), AngleCol, 3);
				}
			}
			cv::imshow(winNamedet,frame);	// dislpay frame
			waitkey.data = cv::waitKey(1);	// key given by the user (http://docs.opencv.org/2.4/modules/highgui/doc/user_interface.html?highlight=waitkey for more info)
			if(waitkey.data!=-1){
				if(waitkey.data == 27){
					shutdownkey.data = true;
					shutdownkey_pub_.publish(shutdownkey);
					ROS_INFO("Shutdown\n");
					ros::Duration(0.06).sleep();
					ros::shutdown();
				}
				else waitkey_pub_.publish(waitkey);
			}
		}
	}
};
//	END Display Image Class
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char **argv)
{
	ros::init(argc, argv, "display_node");
	// construct object of Display Image Class:	
	ImageShow is;
	// Constantly call the callbacks:
	ros::spin();
	return 0;
}
