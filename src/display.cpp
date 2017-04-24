/*
receives frame and displayes them with and without target rectangle
window without target rectangle is used to interrupt with the waitkey
*/
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

std::string winNameraw = "Preview Frame";
std::string winNamedet = "Preview Detection";


class ImageShow{
	ros::NodeHandle nh_;
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
double tside, tsize;
int pixelX, pixelY;
std_msgs::Int8 waitkey;
std_msgs::Bool shutdownkey;
int frWidth = 640;
int frHeight = 360;
public:
	ImageShow() : it_(nh_) {
		image_sub_ = it_.subscribe("/camera/image_raw",1,
			&ImageShow::imageCb, this);
//		pixelpos_sub_ = nh_.subscribe("pixelpos",1, &ImageShow::pixelposCb,this);
		comparedpixelpos_sub_ = nh_.subscribe("comparedpixelpos",1, &ImageShow::comparedpixelposCb,this);
		focus_and_zoom_sub_ = nh_.subscribe("fandzmsg",1,&ImageShow::displayDetCb, this);
		waitkey_pub_ = nh_.advertise<std_msgs::Int8>("waitkey",1);
		shutdownkey_pub_ = nh_.advertise<std_msgs::Bool>("shutdownkey",1);

		cv::namedWindow(winNameraw,cv::WINDOW_KEEPRATIO);
		cv::namedWindow(winNamedet,cv::WINDOW_KEEPRATIO);
	}

	~ImageShow(){
		cv::destroyWindow(winNameraw);
		cv::destroyWindow(winNamedet);
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
		frame = cv_ptr->image;
		if(!cv_ptr->image.empty()){
			cv::imshow(winNameraw,cv_ptr->image);
			waitkey.data = cv::waitKey(1);
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
	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
		pixelX = pixelpos->x;
		pixelY = pixelpos->y;
	}
	void comparedpixelposCb(const geometry_msgs::Pose2D::ConstPtr& comparedpixelpos){
		pixelX = comparedpixelpos->x;
		pixelY = comparedpixelpos->y;
	}
	void displayDetCb(const std_msgs::Float32MultiArray::ConstPtr& fandzmsg){
		if(!frame.empty()){
			tsize = fandzmsg->data[2];
			if(pixelX>=0){
			cv::rectangle(frame, cv::Rect(round(pixelX-tsize/2.0),round(pixelY-tsize/2.0), round(tsize), round(tsize))&cv::Rect(0,0,frWidth-1,frHeight-1),
				cv::Scalar(0,0,255),2,8,0);
			}
			cv::imshow(winNamedet,frame);
			cv::waitKey(1);
		}
	}
	void setframeprops(int width, int height){
		frWidth = width;
		frHeight = height;
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "display_node");
	ImageShow is;
	if(argc == 3){
		int frWidth = atoi(argv[1]);
		int frHeight = atoi(argv[2]);
		is.setframeprops(frWidth,frHeight);
	std::cout << "\nframe width 	" << frWidth
	<< "\nframe height	" << frHeight << "\n"; 
	}
	else{
		std::cout<< "Default frame property values used [640x360].\n You can pass the arguments using: [frame width]  [frame height] in command line\n";
	}
	ros::spin();
	return 0;
}
