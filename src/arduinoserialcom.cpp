//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\
// Purpose:
// 1) Set up serial connection to the arduino
// 2) Zoom control
// 3) Focus control
//
//
// Author: Guetg Fadri, guetgf@student.ethz.ch
// 28.05.2017
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Arduino Communication Class
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class ArduinoCom{

	ros::NodeHandle nh_;
	ros::Publisher zoom_level_pub_;
	ros::Publisher focus_level_pub_;
	ros::Subscriber focus_and_zoom_sub_;
	ros::Subscriber shutdownkey_sub_;
private:
	int serial_fd;		// Arduino handle
	// Focus variables:
	int focus;				// focus level
	int focmin = 650;		// maximum focus level defined by user
	int focmax = 800;		// minimum focus level defined by user
	int focusstep = 3;		// step size of peak search
	int focusscanstep = 10;	// step size of focus scan
	int delay = 3;			// delay in sampling times
	int smalldelay = 3;	
	int largedelay = 2*smalldelay;
	bool
	NEAR = false,			
	FAR = true,								
	detectflag = false, 	
	olddetectflag = false,
	scanfocus = true,		// scan focus mode
	direction = FAR,
	firstscan = true,		// first loop of scan
	detected_during_scan = false;	
	int scanfocmax;			// sharpest focus level measured during scan 
	int notdetectcounter = 0;
	double sharpness, sharpnessold = 0, sharpnessmax = 0, fsharpness, fsharpnessold;

	// Zoom variables:
	int zoomcounter = 0, zoomdelay = 6;
	int  zoom = 171, zoommax = 171, zoommin = 0, zoomstep = 3, oldzoom = zoom;
	float tsize = 100, tsizefiltered = 100; 	// target size
	int tsizeUpperThrshld = 30, tsizeLowerThrshld = 25;	// range of desired target size
	int loopcounter = 0;	// used for timing the focus and zoom controller
	// messages
	std_msgs::Int16 zoomlevel;
	std_msgs::Int16 focuslevel;

public:
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Constructor
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	ArduinoCom(){
		string serial_path = "/dev/ttyACM0";		// serial path to arduino (default)

		zoom_level_pub_ = nh_.advertise<std_msgs::Int16>("zoomlevel",1);
		focus_level_pub_ = nh_.advertise<std_msgs::Int16>("focuslevel",1);
		focus_and_zoom_sub_ = nh_.subscribe("fandzmsg",1,&ArduinoCom::d_f_z_Callback,this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&ArduinoCom::shutdownCb,this);
		
		// Read in parameters that are defined in launch file:
		if(nh_.hasParam("arduinoserial/focmin")){
			bool success = nh_.getParam("arduinoserial/focmin",focmin);
			ROS_INFO("Read ardnuino parameter focmin, success: %d	%d",focmin, success);
		}
		else ROS_INFO("Arduino param focmin not found");
		if(nh_.hasParam("arduinoserial/focmax")){
			bool success = nh_.getParam("arduinoserial/focmax",focmax);
			ROS_INFO("Read ardnuino parameter focmax, success: %d	%d",focmax, success);
		}
		else ROS_INFO("Arduino param focmax not found");	
		if(nh_.hasParam("arduinoserial/focusstep")){
			bool success = nh_.getParam("arduinoserial/focusstep",focusstep);
			ROS_INFO("Read ardnuino parameter focusstep, success: %d	%d",focusstep, success);
		}
		else ROS_INFO("Arduino param focusstep not found");		
		if(nh_.hasParam("arduinoserial/focusscanstep")){
			bool success = nh_.getParam("arduinoserial/focusscanstep",focusscanstep);
			ROS_INFO("Read ardnuino parameter focusscanstep, success: %d	%d",focusscanstep, success);
		}
		else ROS_INFO("Arduino param focusscanstep not found");	
		if(nh_.hasParam("arduinoserial/tsizeLowerThrshld")){
			bool success = nh_.getParam("arduinoserial/tsizeLowerThrshld",tsizeLowerThrshld);
			ROS_INFO("Read ardnuino parameter tsizeLowerThrshld, success: %d	%d",tsizeLowerThrshld, success);
		}
		else ROS_INFO("Arduino param tsizeLowerThrshld not found");
		if(nh_.hasParam("arduinoserial/tsizeUpperThrshld")){
			bool success = nh_.getParam("arduinoserial/tsizeUpperThrshld",tsizeUpperThrshld);
			ROS_INFO("Read ardnuino parameter tsizeUpperThrshld, success: %d	%d",tsizeUpperThrshld, success);
		}
		else ROS_INFO("Arduino param tsizeLowerThrshld not found");
		if(nh_.hasParam("arduinoserial/serial_path")){
			bool success = nh_.getParam("arduinoserial/serial_path",serial_path);
			ROS_INFO("Read ardnuino parameter serial_path, success: %s	%d",serial_path.c_str(), success);
		}
		else ROS_INFO("Arduino param serial_path not found");

		//Setup Serial communication to arduino:
		serial_fd = open(serial_path.c_str(),O_RDWR|O_NOCTTY|O_NDELAY);
		struct termios serial_settings;
		if(serial_fd == -1){
			ROS_INFO("Serial Port connection Failed. - shutdown");
			ros::shutdown();
		}
		else{
			//Get and set serial port settings:
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
			ROS_INFO("Setting up Serial Port settings Failed - shutdown");
			ros::shutdown();
		}
		else
			ROS_INFO("Serial Port connection Success.");
		}
		
		// Set camera settings:
		sleep(3);
		// set gain on maximum:
		ostringstream cmd_buffg;
		cmd_buffg << ">g,"<<16<<"<";
		string cmdg = cmd_buffg.str();
		write(serial_fd,cmdg.c_str(),cmdg.length());
		sleep(1);
		// set shutter speed on 1/360 (inverse*10^6):
		ostringstream cmd_buffe;
		cmd_buffe << ">e,"<<2777<<"<";
		string cmde = cmd_buffe.str();
		write(serial_fd,cmde.c_str(),cmde.length());
		sleep(1);
		// set frame rate:
		ostringstream cmd_buffv;
		cmd_buffv << ">v,"<<60<<","<< 0<<","<< 3 <<","<<0<<","<<0<<"<";
		string cmdv = cmd_buffv.str();
		write(serial_fd,cmdv.c_str(),cmdv.length());
	}
	// END Constructor
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//Destructor:
	~ArduinoCom(){
	}
	// Function for sending focus level to Arduino:
	void sendFocus(){
		ostringstream cmd_buff;
		cmd_buff << ">f,"<<focus<<"<";
		string cmd = cmd_buff.str();
		write(serial_fd,cmd.c_str(),cmd.length());
	}
	// Function for sending zoom level to Arduino:
	void sendZoom(){
		ostringstream cmd_buff;
		cmd_buff << ">z,"<<zoom<<"<";
		string cmd = cmd_buff.str();
		write(serial_fd,cmd.c_str(),cmd.length());
	}

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Focus scan function
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void scanFocus(){
		// initialize focus scan mode:
		if(firstscan){ 
			if((loopcounter%delay == 0)){
				detected_during_scan = false;
				sharpnessmax = 0;
				delay = largedelay;  // larger delay due to larger focus step
				focus = focmin;
				sendFocus();		// Go on minimum focus level
				firstscan = false;
				focuslevel.data = focus;
				focus_level_pub_.publish(focuslevel);
			}
		}
		// Focus scan loop:
		else if((loopcounter%delay == 0)){
			delay = smalldelay;
			// check if marker is detected and if sharpness has increased:
			if((sharpness>sharpnessmax)&detectflag){
				sharpnessmax = sharpness;
				scanfocmax = focus;
				detected_during_scan = true;
			}
			// increase focus level
			focus += focusscanstep;
			// Check if focus scan has reached maximum:
			if(focus>focmax){
				if(detected_during_scan){
					focus = scanfocmax;
					scanfocus = false;
					direction = FAR;
				}
				firstscan = true;
				loopcounter = 0;
			}
			sendFocus();
			focuslevel.data = focus;
			focus_level_pub_.publish(focuslevel);
		}

	}
	// END Focus scan function
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Peak search function
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void controlFocus(){
		if((loopcounter%delay == 0) && olddetectflag){
			fsharpness = sharpness + sharpnessold; // moving average filter
			if(fsharpness<fsharpnessold){
				direction ^= true;	// Change direction of focus step if sharpness has decreased
			}
			if(direction == NEAR) focus -= focusstep;
			else focus += focusstep;
			focus = min(focmax, focus);
			focus = max(focmin, focus);
			fsharpnessold = fsharpness;
			fsharpness = 0;
			sendFocus();
			focuslevel.data = focus;
			focus_level_pub_.publish(focuslevel);
		}
		if(loopcounter>23) loopcounter = 0;
	}
	// END Peak search function
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Zoom control function
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void controlZoom(){
		oldzoom = zoom;
		// zoom out if object has not been detected for a specific time:
		if(!detectflag){
			if(notdetectcounter > 32){
				zoom = zoommax;
				sendZoom();
				loopcounter = 0;
				notdetectcounter = 0;
			}
		}
		else if (loopcounter%8 == 6){ 
			// Check if filtered marker size is outside of range:
			if(tsizefiltered>tsizeUpperThrshld){
				zoom += zoomstep;
				zoom = min(zoom, zoommax);
				sendZoom();
			}
			else if(tsizefiltered<tsizeLowerThrshld){
				zoom -= zoomstep;
				zoom = max(zoom,zoommin);
				sendZoom();
			}
		}
		if(zoom != oldzoom){
			zoomlevel.data = zoom;
			// notify the LQR node if the zoomlevel has changed:
			zoom_level_pub_.publish(zoomlevel);
		}
	}
	// END Zoom control function
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Focus and Zoom message callback
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Calback for receiving fandzmsg.data[0] = detected, [1] = sharpness, [2] = target size
	// This callback applies the zoom and focus control
	void d_f_z_Callback(const std_msgs::Float32MultiArray::ConstPtr& fandzmsg){
		loopcounter++;
		// update detect flag:
		olddetectflag = detectflag;
		detectflag = fandzmsg->data[0];
		//update sharpness:
		if(!olddetectflag && detectflag) sharpnessold = 0;
		else sharpnessold = sharpness;
		sharpness = fandzmsg->data[1];
		// update target size and filter it:
		tsizefiltered = 0.8*tsizefiltered;
		tsize = fandzmsg->data[2];
		tsizefiltered += 0.2*tsize;
		if(detectflag == 0){
			notdetectcounter++;
		}
		else notdetectcounter = 0;
		// apply focus scan if marker has not been detected for a specific time
		if(notdetectcounter>15 && scanfocus == false){ 
			scanfocus = true;
			controlZoom();
			loopcounter = 0;
		}
		else{
			controlZoom();
		}
		if(scanfocus){
			scanFocus();
		}
		else if(detectflag){
			controlFocus();
		}
	}
	// END Focus and Zoom message callback
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Shutdown callback:
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown\n");
		ros::shutdown();
	}

};
// END Arduino Communication Class
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main( int argc, char** argv ) {

	ros::init(argc,argv,"arduinoserialcom_node");
	// construct object of the Arduino communication Class
	ArduinoCom ac;
	ROS_INFO("Arduino is set up successfully");
	// Constantly call the callbacks:
	ros::spin();

	return(0);
}
