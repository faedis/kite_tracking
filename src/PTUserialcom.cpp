//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\
// Purpose:
// 1) Start up the PTU
// 2) Query the position from the PTU and publish them
// 3) Send velocity commands to the PTU
//
//
// Author: Guetg Fadri, guetgf@student.ethz.ch
// 28.05.2017
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\


#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <unistd.h>
#include <sys/time.h>
#include <chrono>
extern "C" {
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
// CPI:
#include "cpiver.h"
#include "ptu.h"
}

using namespace std;


portstream_fd COMstream;	// PTU handle


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PTU Subscribe and Publish Class
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
class PTUSubAndPub{

private:
	ros::NodeHandle n_;
	ros::Publisher ptupos_pub_;
	ros::Subscriber ptucmd_sub_;
	ros::Subscriber waitkey_sub_;
	ros::Subscriber shutdownkey_sub_;
	ros::Subscriber grabbed_sub_;

	geometry_msgs::Pose2D ptupos;

	signed short int panSpeed, tiltSpeed, rehomespeed = 500, panZero = 0, tiltZero=0;
	signed short pPos = 0;		// position of the pan axis, positions
	signed short tPos = 0;		// position of the tilt axis, positions
	signed short* pPtr = &pPos;	// pointer to position of the pan axis, positions
	signed short* tPtr = &tPos;	// pointer to position of the tilt axis, positions
public:
	PTUSubAndPub(){
		// publisher:
		ptupos_pub_ = n_.advertise<geometry_msgs::Pose2D>("ptupos",1);
		// subscriber:
		ptucmd_sub_ = n_.subscribe("ptucmd",1,&PTUSubAndPub::ptucmdCb,this);
		waitkey_sub_ = n_.subscribe("waitkey",1,&PTUSubAndPub::waitkeyCb,this);
		shutdownkey_sub_ = n_.subscribe("shutdownkey",1,&PTUSubAndPub::shutdownCb,this);
		grabbed_sub_ = n_.subscribe("grabbed",1, &PTUSubAndPub::grabbedCb,this);
	}
	// command the PTU speed:
	void ptucmdCb(const geometry_msgs::Pose2D::ConstPtr& ptucmd){
		panSpeed = ptucmd->x;
		tiltSpeed = ptucmd->y;
		char answer = ptu_set_desired_velocities(panSpeed, tiltSpeed);
		if(answer!=0){
			ROS_INFO("error on vel sending:	%c", answer);
		}
	}
	// get PTU position and publish:
	void grabbedCb(const std_msgs::Bool::ConstPtr& grabbed){
		char answer = get_current_positions(pPtr,tPtr);
		if(answer==0){
		ROS_INFO("error on pos reading:	%c", answer);
		}

		ptupos.x = *pPtr;
		ptupos.y = *tPtr;
		ptupos.theta = 0;
		ptupos_pub_.publish(ptupos);

	}
	// Rehome the PTU:
	void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
		int key = waitkey->data;
		if(key == 114){
			ptu_set_desired_velocities(rehomespeed, rehomespeed);
			set_desired_abs_positions(&panZero,&tiltZero);
			await_completion();
		}
	}
	// Shutdown the node:
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ptu_set_desired_velocities(rehomespeed, rehomespeed);
		set_desired_abs_positions(&panZero,&tiltZero);
		await_completion();
		close_host_port(COMstream);				// close connection
		ros::shutdown();
	}
};
// END PTU Subscribe and publish Class
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Main function
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main( int argc, char** argv ) {
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Setting up the PTU
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Set up serial connection to the PTU: 
	int filenumber = 0;
	char COMportName[256] = "/dev/ttyUSB0";
	int BaudRate = 19200;
	set_baud_rate(BaudRate);					// set baud rate
	COMstream = open_host_port(COMportName);	// open serial communication
	if (COMstream == PORT_NOT_OPENED)			// check connection is open, else abord
	{
		printf("\nSerial Port setup error.\n");
		return -1;
	}
	else{
		cout << "Serial port to PTU opened\n\n";
	}
	// When powering up, the PTU has to be calibrated
	cout << "Do you want to reset PTU (suggested when powered up again)\nThen press 1, else any key\n";
	int firstUseInt = 0;
	bool firstUse = false;
	cin >> firstUseInt;
	cout << "\n";
	if(firstUseInt ==1){
		firstUse = true;
		cout<< "Reset starts...\n";
	}
	else{
		firstUse = false;
		cout << "Reset disabled\n";
	}
	// SerialStringOut allows us set more parameter than with the CPI:
	// Check the "Command Reference Manual" for more detail
	if (firstUse) {
		if (SerialStringOut(COMstream, (unsigned char *)"TMH PMH WTH WPQ ") != TRUE) {
			cout << "1st Serial command not sent to PTU \n";
			getchar();
		}
		cout << "Wait until PTU has stopped and then enter a key! \n";
		int dummyvar = 0;
		cin>>dummyvar;
		reset_PTU_parser(2000);
		if (SerialStringOut(COMstream, (unsigned char *)"RD TML PML A TNU-10 LU TXU2250 LU A PNU-2000 LU PXU2000 LU A PU3500 TU3500 TA2000 PA2000 ") != TRUE) { 
			cout << "2nd Serial command not sent to PTU \n";
		}

		reset_PTU_parser(2000);

		if (set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE) == PTU_OK) {
			cout << "Speed mode set to pure velocity \n";
		}
		if (set_mode(COMMAND_EXECUTION_MODE, EXECUTE_IMMEDIATELY) == PTU_OK) {
			cout << "Command execution mode set to immediate execution mode \n";
		}
		signed short int maxPos = 0;
		maxPos = (short)get_current(PAN, MAXIMUM_POSITION);
		cout << "Pan maximum Position " << maxPos << "\n";
		maxPos = 0;
		maxPos = (short)get_current(PAN, MINIMUM_POSITION);
		cout << "Pan minimum Position " << maxPos << "\n";
		maxPos = 0;
		maxPos = (short)get_current(TILT, MAXIMUM_POSITION);
		cout << "Tilt maximum Position " << maxPos << "\n";
		maxPos = 0;
		maxPos = (short)get_current(TILT, MINIMUM_POSITION);
		cout << "Tilt minimum Position " << maxPos << "\n";
		
		signed short int val = 58; 
		if (set_desired(PAN, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Pan base speed could not be set \n";
		}
		val = 58;
		if (set_desired(TILT, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Tilt base speed could not be set \n";
		}
		cout << "Base speed pan: " << (short)get_current(PAN, BASE) << "\n";
		cout << "Base speed tilt: " << (short)get_current(TILT, BASE) << "\n";

		reset_PTU_parser(2000);

		if (SerialStringOut(COMstream, (unsigned char *)"RD TML PML A TNU-10 LU TXU2250 LU A PNU-2000 LU PXU2000 LU A PU3500 TU3500 TA2000 PA2000 ") != TRUE) { //TA50000 PA25000
			cout << "2nd Serial command not sent to PTU \n";
		}

		reset_PTU_parser(2000); 

		if (set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE) == PTU_OK) {
			cout << "Speed mode set to pure velocity \n";
		}
		if (set_mode(COMMAND_EXECUTION_MODE, EXECUTE_IMMEDIATELY) == PTU_OK) {
			cout << "Command execution mode set to immediate execution mode \n";
		}
		maxPos = 0;
		maxPos = (short)get_current(PAN, MAXIMUM_POSITION);
		cout << "Pan maximum Position " << maxPos << "\n";
		maxPos = 0;
		maxPos = (short)get_current(PAN, MINIMUM_POSITION);
		cout << "Pan minimum Position " << maxPos << "\n";
		maxPos = 0;
		maxPos = (short)get_current(TILT, MAXIMUM_POSITION);
		cout << "Tilt maximum Position " << maxPos << "\n";
		maxPos = 0;
		maxPos = (short)get_current(TILT, MINIMUM_POSITION);
		cout << "Tilt minimum Position " << maxPos << "\n";
		
		 val = 58; 
		if (set_desired(PAN, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Pan base speed could not be set \n";
		}
		val = 58;
		if (set_desired(TILT, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Tilt base speed could not be set \n";
		}
		cout << "Base speed pan: " << (short)get_current(PAN, BASE) << "\n";
		cout << "Base speed tilt: " << (short)get_current(TILT, BASE) << "\n";
	}
	// END Setting up the PTU
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	ros::init(argc, argv, "ptuserialcom_node");
	// construct object of the PTU Subscribe and Publish Class:	
	PTUSubAndPub ptusap;
	// Constantly call the callbacks:
	ros::spin();

	return 0;
}
// END Main function
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
