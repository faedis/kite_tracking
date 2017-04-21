/*
Serial connection to ptu
querries ptu position and sends velocity commands
*/

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
extern "C" {
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "cpiver.h"
#include "ptu.h"
}

using namespace std;

portstream_fd COMstream;			// creat PTU handle

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
	signed short pPos = 0;
	signed short tPos = 0;
	signed short* pPtr = &pPos;
	signed short* tPtr = &tPos;
	struct timeval t1, t2;
	double elapsedTime=0;
public:
	PTUSubAndPub(){
		// publisher
		ptupos_pub_ = n_.advertise<geometry_msgs::Pose2D>("ptupos",1);
		// subscriber
		ptucmd_sub_ = n_.subscribe("ptucmd",1,&PTUSubAndPub::ptucmdCb,this);
		waitkey_sub_ = n_.subscribe("waitkey",1,&PTUSubAndPub::waitkeyCb,this);
		shutdownkey_sub_ = n_.subscribe("shutdownkey",1,&PTUSubAndPub::shutdownCb,this);
		grabbed_sub_ = n_.subscribe("grabbed",1, &PTUSubAndPub::grabbedCb,this);
	}
	// set PTU speed
	void ptucmdCb(const geometry_msgs::Pose2D::ConstPtr& ptucmd){
		panSpeed = ptucmd->x;
		tiltSpeed = ptucmd->y;
		//gettimeofday(&t1, NULL);
//		set_desired(PAN, SPEED, &panSpeed, ABSOLUTE);

		char answer = ptu_set_desired_velocities(panSpeed, tiltSpeed);
		if(answer!=0){
			ROS_INFO("error on vel sending:	%c", answer);
		}

//		ROS_INFO("velocity cmd:	%d", panSpeed);
		//gettimeofday(&t2, NULL);
		//elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		//elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		//ROS_INFO("speedcmd %f",elapsedTime);
	}
	// get PTU position and publish
	void grabbedCb(const std_msgs::Bool::ConstPtr& grabbed){
		//ROS_INFO("start query");
/*		gettimeofday(&t1, NULL);
		elapsedTime = 0;
		while(elapsedTime<5){
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		}
*/
/*
		gettimeofday(&t1, NULL);
		while(elapsedTime<7.5){
			gettimeofday(&t2,NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
		}
*/
		char answer = get_current_positions(pPtr,tPtr);
		if(answer==0){
		ROS_INFO("error on pos reading:	%c", answer);
		}
		ptupos.x = *pPtr;
		ptupos.y = *tPtr;
		ptupos.theta = 0;
		ptupos_pub_.publish(ptupos);


//		ROS_INFO("pos query:	%d", *pPtr);
		//ROS_INFO("end query ");
	}
	// rehome or shutdown
	void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
		int key = waitkey->data;
		if(key == 114){
			ptu_set_desired_velocities(rehomespeed, rehomespeed);
			set_desired_abs_positions(&panZero,&tiltZero);
			await_completion();
		}
	}
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ptu_set_desired_velocities(rehomespeed, rehomespeed);
		set_desired_abs_positions(&panZero,&tiltZero);
		await_completion();
		close_host_port(COMstream);				// close connection
		ros::shutdown();
	}
};


int main( int argc, char** argv ) {
	// Set up PTU +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	int filenumber = 0;
	char COMportName[256] = "/dev/ttyUSB0";
	int BaudRate = 19200;
	// Initialize PTU and set mode

	set_baud_rate(BaudRate);			// set baud rate
	COMstream = open_host_port(COMportName);	// open serial communication
	if (COMstream == PORT_NOT_OPENED)		// check connection is open, else abord
	{
		printf("\nSerial Port setup error.\n");
		return -1;
	}
	else{
		cout << "Serial port to PTU opened\n\n";
	}
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
	//Serial can be used for setting user position limits,
	//move power and step mode (e.g. wth = tilt half step mode)
	//wta/wpa is automatic step mode and leads to resolution of
	// 46.2857 rsp 11.5714 arc sec pp, tmr is tilt regular move power
	if (firstUse) {
		if (SerialStringOut(COMstream, (unsigned char *)"TMH PMH WTH WPQ ") != TRUE) {
			cout << "1st Serial command not sent to PTU \n";
			getchar();
		}
		cout << "Wait until PTU has stopped and then enter a key! \n";
		int dummyvar = 0;
		cin>>dummyvar;		// tilt reg move power, tilt min pos, t max pos, p m pos,
		// p x pos, user limits enabled, disable reset on restart,
		//t acc, p acc, p upper speed limit, t u s l
		if (SerialStringOut(COMstream, (unsigned char *)"RD TML PML A TNU-10 LU TXU2250 LU A PNU-2000 LU PXU2000 LU A TA25000 PA12500 PU5000 TU5000 ") != TRUE) { //TA50000 PA25000
			cout << "2nd Serial command not sent to PTU \n";
		}

		reset_PTU_parser(2000); // needed for changing between direct serial comm and cpi commands
								// set pure velocity mode
/*		if (set_mode(POSITION_LIMITS_MODE,ON_MODE) == PTU_OK){
			cout << "Position limits enabled \n";
		}
*/
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
		//  base speed:
		
		signed short int val = 58; 
		if (set_desired(PAN, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Pan base speed could not be set \n";
		}
		val = val;
		if (set_desired(TILT, BASE, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
			cout << "Tilt base speed could not be set \n";
		}
		cout << "Base speed pan: " << (short)get_current(PAN, BASE) << "\n";
		cout << "Base speed tilt: " << (short)get_current(TILT, BASE) << "\n";
		//// acceleration
		//val = 200000;
		//if (set_desired(PAN, ACCELERATION, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Acceleration pan could not be set!\n";
		//}
		//if (set_desired(TILT, ACCELERATION, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Acceleration pan could not be set!\n";
		//}
		//// speed limits
		//val = 10000;
		//if (set_desired(PAN, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Upper speed limit pan could not be set!\n";
		//}
		//if (set_desired(TILT, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&val, ABSOLUTE) != PTU_OK) {
		//	cout << "Upper speed limit pan could not be set!\n";
		//}
	}


	ros::init(argc, argv, "ptuserialcom_node");
	PTUSubAndPub ptusap;
	ros::spin();


	// rehome PTU
	//val = 1000;
	//set_desired(PAN, SPEED, &val, ABSOLUTE);
	//set_desired(TILT, SPEED, &val, ABSOLUTE);
	//val = 0;
	//set_desired(PAN, POSITION, &val, ABSOLUTE);
	//val = 0;
	//set_desired(TILT, POSITION, &val, ABSOLUTE);
	//close_host_port(COMstream);				// close connection


	return 0;
}
