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
#include <std_msgs/Int8.h>
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

class ArduinoCom{

	ros::NodeHandle nh_;
	ros::Publisher zoom_level_pub_;
	ros::Publisher focus_level_pub_;
	ros::Subscriber focus_and_zoom_sub_;
	ros::Subscriber shutdownkey_sub_;
private:
	int serial_fd;
	int fps = 60; 
	// Focus variables
	int focus;
	int focmin = 650;
	int focmax = 800;
	int focusstep = 3;
	int focusscanstep = 10;
	int delay = 3;
	int smalldelay = 3;
	int largedelay = 2*smalldelay;
	bool
	NEAR = false,
	FAR = true,
	OFF = false,
	ON = true,
	detectflag = false, 
	olddetectflag = false,
	scanfocus = true,
	direction = FAR,
	firstscan = true,
	detected_during_scan = false;
	
	int scanfocmax;
	int focuscounter = 0;
	int notdetectcounter = 0;
	double sharpness, sharpnessold = 0, sharpnessmax = 0, fsharpness, fsharpnessold;
	// Zoom variables
	int zoomcounter = 0, zoomdelay = 6;
	int  zoom = 171, zoommax = 171, zoommin = 0, zoomstep = 3, oldzoom = zoom;
	float tsize = 100, tsizefiltered = 100; // target size
	int tsizeUpperThrshld = 30, tsizeLowerThrshld = 25;
	int loopcounter = 0;
	std_msgs::Int16 zoomlevel;
	std_msgs::Int16 focuslevel;

public:
	// constructor
	ArduinoCom(){
		string serial_path = "/dev/ttyACM0";

		zoom_level_pub_ = nh_.advertise<std_msgs::Int16>("zoomlevel",1);
		focus_level_pub_ = nh_.advertise<std_msgs::Int16>("focuslevel",1);
		focus_and_zoom_sub_ = nh_.subscribe("fandzmsg",1,&ArduinoCom::d_f_z_Callback,this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&ArduinoCom::shutdownCb,this);
		
		// read parameters
		if(nh_.hasParam("camera/fps")){
			bool success = nh_.getParam("camera/fps",fps);
			ROS_INFO("Read arduino parameter success, fps: %d	%d",fps, success);
		}
		else ROS_INFO("LQR param not found");
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

		//Setup Serial communication to arduino
		serial_fd = open(serial_path.c_str(),O_RDWR|O_NOCTTY|O_NDELAY);
		struct termios serial_settings;
		// Try opening serial port
		// int serial_fd = open(SERIAL_PATH,O_WRONLY|O_NOCTTY);//serial_fd = open(SERIAL_PATH,O_RDWR|O_NOCTTY);
		if(serial_fd == -1){
			ROS_INFO("Serial Port connection Failed. - shutdown");
			ros::shutdown();
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
			ROS_INFO("Setting up Serial Port settings Failed - shutdown");
			ros::shutdown();
		}
		else
			ROS_INFO("Serial Port connection Success.");
		}
		

		sleep(3); // set gain on maximum
		ostringstream cmd_buffg;
		cmd_buffg << ">g,"<<16<<"<";
		string cmdg = cmd_buffg.str();
		write(serial_fd,cmdg.c_str(),cmdg.length());
		sleep(1); // set shutter speed on 1/360 (inverse*10^6)
		ostringstream cmd_buffe;
		cmd_buffe << ">e,"<<2777<<"<";
		string cmde = cmd_buffe.str();
		write(serial_fd,cmde.c_str(),cmde.length());
		sleep(1);
		ostringstream cmd_buffv;
		cmd_buffv << ">v,"<<60<<","<< 0<<","<< 3 <<","<<0<<","<<0<<"<";
		string cmdv = cmd_buffv.str();
		write(serial_fd,cmdv.c_str(),cmdv.length());

	}
	~ArduinoCom(){
	}

	void sendFocus(){
//		ROS_INFO("focus sent: %d", focus);
		ostringstream cmd_buff;
		cmd_buff << ">f,"<<focus<<"<";
		string cmd = cmd_buff.str();
		write(serial_fd,cmd.c_str(),cmd.length());
	//	ROS_INFO("Sent focus: [%i], Sharpness [%f]", focus,sharpness);
		//cout<<"Sent focus command: "<<cmd<<endl;
	}
	void sendZoom(){
//		ROS_INFO("Zoom sent:	%d",zoom);
		ostringstream cmd_buff;
		cmd_buff << ">z,"<<zoom<<"<";
		string cmd = cmd_buff.str();
		write(serial_fd,cmd.c_str(),cmd.length());
		//ROS_INFO("Sent zoom: [%i]", zoom);
		//cout<<"Sent focus command: "<<cmd<<endl;
	}

	void scanFocus(){
		if(firstscan){
//	ROS_INFO("FIRSTSCAN");
			if((loopcounter%delay == 0)){//(delay+1))>delay-1){
				detected_during_scan = false;
				sharpnessmax = 0;
				delay = largedelay;
				focus = focmin;
				sendFocus();
				firstscan = false;
				focuslevel.data = focus;
				focus_level_pub_.publish(focuslevel);
			}
		}
		else if((loopcounter%delay == 0)){//(delay+1))>delay-1){ 
			delay = smalldelay;
			if((sharpness>sharpnessmax)&detectflag){
				sharpnessmax = sharpness;
				scanfocmax = focus;
				detected_during_scan = true;
			}
			focus += focusscanstep;
			if(focus>focmax){
				if(detected_during_scan){
					focus = scanfocmax;
					scanfocus = false;
					direction = FAR;
				}
				firstscan = true;
				loopcounter = 0;
			}
/*			if(detectflag){
				scanfocus = false;
				firstscan = true;
				direction = FAR;
				loopcounter = 0;
	//			focus -= focusscanstep;
			}*/
//	ROS_INFO("scan");
			sendFocus();
			focuslevel.data = focus;
			focus_level_pub_.publish(focuslevel);
		}

	}

	void controlFocus(){
		if((loopcounter%delay == 0) && olddetectflag){
			fsharpness = sharpness + sharpnessold;
			if(fsharpness<fsharpnessold){
				direction ^= true;
			}
			if(direction == NEAR) focus -= focusstep;
			else focus += focusstep;
			focus = min(focmax, focus);
			focus = max(focmin, focus);
			fsharpnessold = fsharpness;
//		ROS_INFO("%d	%f	%f	%f",focus,sharpness,fsharpness,tsizefiltered);
			fsharpness = 0;
			sendFocus();
			focuslevel.data = focus;
			focus_level_pub_.publish(focuslevel);
		}
		if(loopcounter>23) loopcounter = 0;
	}

	void controlZoom(){
		oldzoom = zoom;
		if(!detectflag){
			if(notdetectcounter > 32){
				zoom = zoommax;
				sendZoom();
				loopcounter = 0;
				notdetectcounter = 0;
			}
		}
		else if (loopcounter%8 == 6){ //22
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
			zoom_level_pub_.publish(zoomlevel);
		}
	}

	void d_f_z_Callback(const std_msgs::Float32MultiArray::ConstPtr& fandzmsg){
		loopcounter++;
		// update detect flag
		olddetectflag = detectflag;
		detectflag = fandzmsg->data[0];
		//update sharpness
		if(!olddetectflag && detectflag) sharpnessold = 0;
		else sharpnessold = sharpness;
		sharpness = fandzmsg->data[1];
		// update target size and filter it
		tsizefiltered = 0.8*tsizefiltered;
		tsize = fandzmsg->data[2];
		tsizefiltered += 0.2*tsize;
//		ROS_INFO("tsize, tsizef: %f	%f",tsize,tsizefiltered);
		// control them
	//	ROS_INFO("detectflag %d",detectflag);
		if(detectflag == 0){
			notdetectcounter++;
		}
		else notdetectcounter = 0;
		if(notdetectcounter>15 && scanfocus == false){ //>10
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

//		cout << tsize << "," << tsizefiltered << "\n";
	}

	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown\n");
		ros::shutdown();
	}
};

int main( int argc, char** argv ) {

	ROS_INFO("before init");
	ros::init(argc,argv,"arduinoserialcom_node");
	ROS_INFO("after init");
	ArduinoCom ac;
	ROS_INFO("after class");
	ros::spin();

	return(0);
}
