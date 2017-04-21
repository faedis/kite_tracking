///////////////////////////////////////////////////////////////////////////
// This Program controls focus and zoom
// The range is only of 100 focus steps. 
// Min to Max kite position require focus 900 to focus 1000 (50m to infinity)

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
//#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2//objdetect.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/tracking.hpp"
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <cstring>
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

#define BAUDRATE B9600
#define SERIAL_PATH "/dev/ttyACM0"
//Setup Serial communication
struct termios serial_settings;
// Try opening serial port
// int serial_fd = open(SERIAL_PATH,O_WRONLY|O_NOCTTY);//serial_fd = open(SERIAL_PATH,O_RDWR|O_NOCTTY);
int serial_fd;


#define NEAR 0
#define FAR 1
#define OFF 0
#define ON 1

// Focus variables
int focus;
int focmin = 650;
int focmax = 800;
int focrange = 100;
int focrangecenter;
int focusstep = 3;
int focusscanstep = 10;
int delay;
int smalldelay = 6;
int largedelay = 2*delay;
bool detectflag = false, knowcenterflag = false, oldknowcenterflag = false, olddetectflag = false, scanfocus = true, direction = FAR,firstscan = true;
int scanfocmax;
int focuscounter = 0;
int notdetectcounter = 0;
double sharpness, sharpnessold = 0, fsharpness, fsharpnessold;
// Zoom variables
int zoomcounter = 0, zoomdelay = 6;
int  zoom = 170, zoommax = 170, zoommin = 0, zoomstep = 3;
float tsize, tsizefiltered; // target size
float tsizeUpperThrshld = 30, tsizeLowerThrshld = 25;
int loopcounter = 0;

void sendFocus(){
	ostringstream cmd_buff;
	cmd_buff << ">f,"<<focus<<"<";
	string cmd = cmd_buff.str();
	write(serial_fd,cmd.c_str(),cmd.length());
//	ROS_INFO("Sent focus: [%i], Sharpness [%f]", focus,sharpness);
	//cout<<"Sent focus command: "<<cmd<<endl;
}
void sendZoom(){
	ostringstream cmd_buff;
	cmd_buff << ">z,"<<zoom<<"<";
	string cmd = cmd_buff.str();
	write(serial_fd,cmd.c_str(),cmd.length());
	//ROS_INFO("Sent zoom: [%i]", zoom);
	//cout<<"Sent focus command: "<<cmd<<endl;
}

void scanFocus(){
	if(firstscan){
		loopcounter = 0;
		delay = largedelay;
		focus = focmin;
		sendFocus();
		firstscan = false;
	}
	else if(loopcounter>delay){
		delay = smalldelay;
		loopcounter = 0;
		if(sharpness>sharpnessold){
			scanfocmax = focus;
		}
		focus += focusscanstep;
		if(focus>focmax){
			scanfocus = false;
			firstscan = true;
			focus = scanfocmax;
			loopcounter = 0;
		}
		if(detectflag){
			scanfocus = false;
			firstscan = true;
			direction = FAR;
			loopcounter = 0;
//			focus -= focusscanstep;
		}
		sendFocus();
	}

}

void controlFocus(){
	fsharpness += sharpness;
	if(loopcounter>smalldelay){
		fsharpness /= (smalldelay+2);
		loopcounter = 0;
		if(fsharpness<fsharpnessold){
			direction ^= true;
		}
		if(direction == NEAR) focus -= focusstep;
		else focus += focusstep;
		focus = min(focmax, focus);
		focus = max(focmin, focus);
		cout << "focus control value		" << focus << "\n";
		fsharpnessold = fsharpness;
		fsharpness = 0;
		sendFocus();
	}
}

void controlZoom(){
	if(!detectflag){
		if(notdetectcounter > 9){
			zoom = zoommax;
			sendZoom();
		}
	}
	else if (loopcounter==4){
		if(tsizefiltered>tsizeUpperThrshld){
			zoom += zoomstep;
			zoom = max(zoom, zoommin);
			sendZoom();
		}
		else if(tsizefiltered<tsizeLowerThrshld){
			zoom -= zoomstep;
			zoom = min(zoom,zoommax);
			sendZoom();
		}
		zoomcounter = 0;
		ROS_INFO("Zoom:	%d",zoom);
	}
}

void d_f_z_Callback(const std_msgs::Float32MultiArray::ConstPtr& fandzmsg){
	loopcounter++;
	// update knowcenterflag
	oldknowcenterflag = knowcenterflag;
	knowcenterflag = fandzmsg->data[3];
	// update detect flag
	olddetectflag = detectflag;
	detectflag = fandzmsg->data[0];
	//update sharpness
	if(!oldknowcenterflag && knowcenterflag) sharpnessold = 0;
	else sharpnessold = sharpness;
	sharpness = fandzmsg->data[1];
	// update zoom
	tsizefiltered = 0.8*tsizefiltered;
	tsize = fandzmsg->data[2];
	tsizefiltered += 0.2*tsize;
	// control them
//	ROS_INFO("detectflag %d",detectflag);
	if(detectflag == 0){
		notdetectcounter++;
	}
	else notdetectcounter = 0;
	if(notdetectcounter>10){
		scanfocus = true;
		notdetectcounter = 0;
	}
	if(scanfocus){
		scanFocus();
	}
	else controlFocus();

	controlZoom();
}

void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
	ROS_INFO("Shutdown\n");
	ros::shutdown();
}

int main( int argc, char** argv ) {
	// Open serial port to arduino++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	serial_fd = open(SERIAL_PATH,O_RDWR|O_NOCTTY|O_NDELAY);
	if(serial_fd == -1){
		printf("Serial Port connection Failed.\n");
		return -1;
	}
	else{
		//Get serial port settings
		tcgetattr(serial_fd, &serial_settings); //Get Current Settings of the Port
		cfsetispeed(&serial_settings,BAUDRATE); //Set Input Baudrate
		cfsetospeed(&serial_settings,BAUDRATE); //Set Output Baudrate
		serial_settings.c_cflag &= ~PARENB; //Mask Parity Bit as No Parity
		serial_settings.c_cflag &= ~CSTOPB; //Set Stop Bits as 1 or else it will be 2
		serial_settings.c_cflag &= ~CSIZE; //Clear the current no. of data bit setting
		serial_settings.c_cflag |= CS8; //Set no. of data bits as 8 Bits

		serial_settings.c_iflag = 0; //Mask Parity Bit as No Parity
		serial_settings.c_oflag = 0;
		serial_settings.c_lflag = 0;
	if(tcsetattr(serial_fd, TCSAFLUSH, &serial_settings) < 0){
		printf("Setting up Serial Port settings Failed");
		return -1;
	}
	else
		printf("Serial Port connection Success.\n\n");
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	ros::init(argc,argv,"arduinoserialcom_node");

	ros::NodeHandle n_;
	
	ros::Subscriber focus_and_zoom_sub_ = n_.subscribe("fandzmsg",1,d_f_z_Callback);
	ros::Subscriber shutdownkey_sub_ = n_.subscribe("shutdownkey",1,shutdownCb);
	ros::spin();

	return(0);
}

