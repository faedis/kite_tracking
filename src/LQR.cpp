//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\
// Purpose:
// 1) combining measurements of pan position and pixel position of kite
// 2) calculate the control input for the ptu
// 3) update the angle of view
// 4) write data of variables to a textfile
//
//
// Author: Guetg Fadri, guetgf@student.ethz.ch
// 28.05.2017
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\\

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <sys/time.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <map>
#include <iterator>

// Generating artificial signals:
// These signals can be assigned in main function
//Specific input signal
std::vector<double> uspec;
//Specific reference signal
std::vector<double> rspec;
int speccount = 0;



// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Kalman Filter Class
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// simple first order model
// based on https://ch.mathworks.com/help/control/ug/kalman-filtering.html
class Kalman{
private:
	double
	x1_hat,
	ox1_hat,
	x2_hat,
	ox2_hat,
	x_hat,
	Ts,
	M1,
	M2,
	a11,
	a12,
	a13,
	a21,
	a22,
	a23,
	a31,
	a32;
public:
	void kalmanInit(double, double, double);
	void kalman(double);
	double getstate1(){
		return this -> x1_hat;
	}
	double getstate2(){
		return this -> x2_hat;
	}
	double getostate1(){
		return this -> ox1_hat;
	}
	double getostate2(){
		return this -> ox2_hat;
	}
	double getestimate(){
		return this -> x_hat;
	}
};
// initialization
void Kalman::kalmanInit(double ts, double m1, double m2){
	this -> Ts = ts;
	this -> M1 = m1;
	this -> M2 = m2;
	this -> x1_hat = 0;
	this -> x2_hat = 0;
	this -> x_hat = 0;
	this -> a11 = 1 - M1 - Ts*M2;
	this -> a12 = Ts;
	this -> a13 = M1 + Ts*M2;
	this -> a21 = -M2;
	this -> a22 = 1;
	this -> a23 = M2;
	this -> a31 = 1 - M1;
	this -> a32 = M1;
}
// filter update
void Kalman::kalman(double measurement){
	this -> ox1_hat = this -> x1_hat;
	this -> ox2_hat = this -> x2_hat;
	this -> x1_hat  = a11 * this -> ox1_hat + a12 * this -> ox2_hat + a13 * measurement;
	this -> x2_hat  = a21 * this -> ox1_hat + a22 * this -> ox2_hat + a23 * measurement;
	this -> x_hat   = a31 * this -> ox1_hat + a32 * measurement; 
}
// END Kalman Filter Class
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	LQR Class
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contains all work of this node

class LQR{
	// The ROS subscribers and publishers:
	ros::NodeHandle nh_;
	ros::Subscriber pixelpos_sub_;
	ros::Subscriber comparedpixelpos_sub_;
	ros::Subscriber waitkey_sub_;
	ros::Subscriber shutdownkey_sub_;
	ros::Subscriber ptupos_sub_;
	ros::Subscriber zoomlevel_sub_;
	ros::Publisher ptucmd_pub_;
	ros::Publisher targetpos_pub_;
private:
// The publisher meassages:
geometry_msgs::Pose2D ptucmd;
geometry_msgs::Pose2D targetpos;
// Kalman objects:
Kalman
	krx,
	kry,
	kx,
	ky;
// Angle of view mapping variables:
std::map<int,double> mapAngleOfViewX;
std::map<int,double> mapAngleOfViewY;
// Variables for time measurement:
struct timeval t1, t2, t3,t4;
double elapsedTime;
// Boolean Variables for Control loop:
bool errorarrived = false, ptuposarrived = false;
bool firstloop = true, secondloop = true, thirdloop = true;
bool PTUctrl = false;
bool detected = false;
// Angle of view in phi and theta direction:
double alphaX = 0.079781;
double alphaY = 0.044942;
// Path to file containing mapping of angle of view:
std::string mapFilePath = "/home/guetgf/catkin_ws/src/kite_tracking/src/angleOfViewLabScale.txt";
// Resolution of pan and tilt axis, rad/position:
double pRes = 92.5714 / 3600.0 * M_PI / 180.0;
double tRes = 46.2857 / 3600.0 * M_PI / 180.0;
// Frame width and height:
int frWidth = 640;
int frHeight = 360;
// Center coordinates of frame:
int hWidth = frWidth/2;
int hHeight = frHeight/2;
// PTU limits:
int uXmax= 3500, uXmin = 60;			// pan input limits pos/second
int uYmax =3500, uYmin = 60;			// tilt input limits pos/second
int deltaUXmax  = 250;					// max pan input change rate limit pos/Ts
int deltaUYmax = 250;					// max tilt input change rate limit pos/Ts
// tolerance of controller in pixel:
int tolerance = 6;						

// Variables for collecting and writing data to text file:
std::ofstream myfile;
int filenumber = 0;
bool collectdata = false, writedata = false;
std::vector<double> timeVec;
std::vector<double> exVec;
std::vector<double> eyVec;
std::vector<double> uxVec;
std::vector<double> uyVec;
std::vector<double> panposVec;
std::vector<double> tiltposVec;
std::vector<double> targetX;
std::vector<double> targetY;
std::vector<double> efxVec;
std::vector<double> timePosVec;
std::vector<double> xhatVec;
std::vector<double> x1hatVec;
std::vector<double> x2hatVec;
std::vector<double> yhatVec;
std::vector<double> y1hatVec;
std::vector<double> y2hatVec;
std::vector<double> rxhatVec;
std::vector<double> rx1hatVec;
std::vector<double> rx2hatVec;
std::vector<double> ryhatVec;
std::vector<double> ry1hatVec;
std::vector<double> ry2hatVec;
// 
int key = -1;
// Measurement and controller variables
double 
pixelX = 0,				// local error phi, pixel
pixelY = 0,				// local error theta, pixel
eX = 0,					// local error pan, rad
eY = 0,					// local error tilt, rad
yX = 0,					// ptu pan position, rad
oyX = 0,				// old ptu pan position, rad
yY = 0,					// ptu titl position, rad
oyY = 0,				// old ptu tilt position, rad
rX = 0,					// target position phi, rad
orX = 0,				// old target position phi, rad		
rY = 0,					// target position theta, rad
orY = 0,				// old target position theta, rad
uXrad = 0,				// input pan, rad/second
ouXrad = 0,				// old input pan, rad/second 
uYrad = 0,				// input tilt, rad/second
ouYrad = 0,				// old input tilt, rad/second
uX = 0,					// input pan, positions/second
uY = 0,					// input tilt, positions/second
ouX = 0,				// old input pan, positions/second
ouY = 0;				// old input tilt, positions/second
double
K1 = 4.9159,			// LQR gain K_1
K2 = 1.0256;			// LQR gain K_2
int fps = 30;			// frames per second
double Ts = 2.0/(double)fps;	// sampling time

// kalman filter parameters:
double
Mx1 = 0.9704,	
Mx2 = 13.7185,
My1 = Mx1,
My2 = Mx2;





public:
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//	Constructor
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	LQR(){
		// initialize subscribers and putlishers:
		pixelpos_sub_ = nh_.subscribe("pixelpos",1,&LQR::pixelposCb,this);
		waitkey_sub_  = nh_.subscribe("waitkey",1,&LQR::waitkeyCb,this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&LQR::shutdownCb,this);
		ptupos_sub_ = nh_.subscribe("ptupos",1,&LQR::ptuposCb,this);
		zoomlevel_sub_ = nh_.subscribe("zoomlevel",1,&LQR::zoomlevelCb,this);
		ptucmd_pub_ = nh_.advertise<geometry_msgs::Pose2D>("ptucmd",1);
		targetpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("targetpos",1);
		// read from ros parameter server:
		// these parameters can be set in the launch file
		if(nh_.hasParam("camera/frWidth")){
			bool success = nh_.getParam("camera/frWidth",frWidth);
			ROS_INFO("Read LQR parameter frWidth, success: %d	%d",frWidth, success);
		}
		else ROS_INFO("LQR param not found");
		if(nh_.hasParam("camera/fps")){
			bool success = nh_.getParam("camera/fps",fps);
			ROS_INFO("Read LQR parameter fps, success: %d	%d",fps, success);
		}
		else ROS_INFO("LQR param not found");
		if(nh_.hasParam("camera/frHeight")){
			bool success = nh_.getParam("camera/frHeight",frHeight);
			ROS_INFO("Read LQR parameter frHeight, success: %d	%d",frHeight, success);
		}
		else ROS_INFO("LQR param height not found");
		if(nh_.hasParam("lqr/pRes")){
			bool success = nh_.getParam("lqr/pRes",pRes);
			ROS_INFO("Read LQR parameter pRes, success: %f	%d",pRes, success);
		}
		else ROS_INFO("LQR param pRes not found");
		if(nh_.hasParam("lqr/tRes")){
			bool success = nh_.getParam("lqr/tRes",tRes);
			ROS_INFO("LQR Read parameter tRes, success: %f	%d",tRes, success);
		}
		else ROS_INFO("LQR param tRes not found");
		if(nh_.hasParam("lqr/tolerance")){
			bool success = nh_.getParam("lqr/tolerance",tolerance);
			ROS_INFO("Read LQR parameter tolerance, success: %d	%d",tolerance, success);
		}
		else ROS_INFO("LQR param tolerance not found");
		if(nh_.hasParam("lqr/K1")){
			bool success = nh_.getParam("lqr/K1",K1);
			ROS_INFO("Read LQR parameter K1, success: %f	%d",K1, success);
		}
		else ROS_INFO("LQR param K1 not found");
		if(nh_.hasParam("lqr/K2")){
			bool success = nh_.getParam("lqr/K2",K2);
			ROS_INFO("Read LQR parameter K2, success: %f	%d",K2, success);
		}
		else ROS_INFO("LQR param K2 not found");
		if(nh_.hasParam("lqr/Mx1")){
			bool success = nh_.getParam("lqr/Mx1",Mx1);
			ROS_INFO("Read LQRparameter Mx1, success: %f	%d",Mx1, success);
		}
		else ROS_INFO("LQR param Mx1 not found");
		if(nh_.hasParam("lqr/Mx2")){
			bool success = nh_.getParam("lqr/Mx2",Mx2);
			ROS_INFO("Read LQR parameter Mx2, success: %f	%d",Mx2, success);
		}
		else ROS_INFO("LQR param Mx2 not found");
		if(nh_.hasParam("lqr/My1")){
			bool success = nh_.getParam("lqr/My1",My1);
			ROS_INFO("Read LQR parameter My1, success: %f	%d",My1, success);
		}
		else ROS_INFO("LQR param My1 not found");
		if(nh_.hasParam("lqr/My2")){
			bool success = nh_.getParam("lqr/My2",My2);
			ROS_INFO("Read LQR parameter My2, success: %f	%d",My2, success);
		}
		else ROS_INFO("LQR param My2 not found");
		if(nh_.hasParam("lqr/mapFilePath")){
			bool success = nh_.getParam("lqr/mapFilePath",mapFilePath);
			ROS_INFO("Read LQR parameter mapFile, success: %s	%d",mapFilePath.c_str(), success);
		}
		else ROS_INFO("LQR param mapFilePath not found");
		// read in angle of view map text file:
		std::vector<int> zoomLevelVec;
		std::vector<double> angleOfViewXVec, angleOfViewYVec;
		int tempz;
		double tempx, tempy;
		std::ifstream inputfile(mapFilePath.c_str());
		if (inputfile.is_open()){
			while (inputfile >> tempz){
				inputfile >> tempx;
				inputfile >> tempy;
				zoomLevelVec.push_back(tempz);
				angleOfViewXVec.push_back(tempx);
				angleOfViewYVec.push_back(tempy);
			}
			inputfile.close();
		}
		else{
			ROS_INFO("Unable to open file\n shutdown");
			ros::shutdown();
		}
		// Initialize Kalman filter:
		kx.kalmanInit(Ts, Mx1, Mx2);
		ky.kalmanInit(Ts, My1, My2);
		krx.kalmanInit(Ts, Mx1, Mx2);
		kry.kalmanInit(Ts,My1, My2);

		// Assign angle of view map:
		for(int i = 0; i<zoomLevelVec.size(); i++){
			mapAngleOfViewX.insert(std::make_pair(zoomLevelVec[i], angleOfViewXVec[i]));
			mapAngleOfViewY.insert(std::make_pair(zoomLevelVec[i], angleOfViewYVec[i]));
		}
		zoomLevelVec.clear();
		angleOfViewXVec.clear();
		angleOfViewYVec.clear();
	}
	//	END Constructor
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Destructor:
	~LQR(){
	}

	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//	Pixel position callback
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
		errorarrived = true; // flag that pixel have arrived
		// Assign new pixel position of target:
		pixelX = pixelpos->x;
		pixelY = pixelpos->y;
		// Transform into radians:
		if(pixelX>=0){	// check if target has been detected
		eX = -alphaX * (pixelX - (double)hWidth) / (double)hWidth;
		eY = -alphaY * (pixelY - (double)hHeight) / (double)hHeight;
		detected = true;
		}
		else detected = false;

		// update and publish target position if all information has arrvied
		if(ptuposarrived){	// Check if position measurement of PTU has arrived
			orX = rX;
			orY = rY;
			if(pixelX>=0){
				rX = oyX+eX; // take camera delay into account
				rY = oyY+eY;
			}
			else{
				rX = 0.95*rX;
				rY = 0.95*rY;
				pixelX = (double)hWidth;
				pixelY = (double)hHeight;
			}
			ptuposarrived = false;
			targetpos.x = rX;
			targetpos.y = rY;
			targetpos.theta = 0;
			targetpos_pub_.publish(targetpos);

/*
// Uncomment for feeding artificial reference signal
			if(PTUctrl){
			// Specific Reference Signal:
				if(speccount<rspec.size()){
					rY = 0;
					rX = rspec[speccount];
					eX = rX - yX;
					eY = rY - yY;
				}
				else{
					rX = orX;
					rY = orY;
				}
			rY = 0;
			eY = 0;
			speccount++;
			}
*/
			// Kalman filter update:
			krx.kalman(rX);
			kry.kalman(rY);
			kx.kalman(oyX); // take camera delay into account -> oyX instead of yX
			ky.kalman(oyY);
		}
		// !!Backup!! if position from PTU has not arrived:
		// has not been observed yet with 15Hz loop rate
		else{ 
			oyX = yX;
			oyY = yY;
			yX = kx.getstate1(); // use prediction instead of missing measurement
			yY = ky.getstate1();
			orX = rX;
			orY = rY;
			if(pixelX>=0){
				rX = oyX+eX; // take camera delay into account
				rY = oyY+eY;
			}
			else{
				rX = 0.95*rX;
				rY = 0.95*rY;
				pixelX = (double)hWidth;
				pixelY = (double)hHeight;
			}

/*
// Uncomment for feeding artificial reference signal
			if(PTUctrl){
			// Specific Reference Signal:
				if(speccount<rspec.size()){
					rY = 0;
					rX = rspec[speccount];
					eX = rX - yX;
					eY = rY - yY;
				}
				else{
					rX = orX;
					rY = orY;
				}
			rY = 0;
			eY = 0;
			speccount++;
			}

*/
			ptuposarrived = false;
			targetpos.x = rX;
			targetpos.y = rY;
			targetpos.theta = 0;
			targetpos_pub_.publish(targetpos);	
			// update:
			// Reference
			krx.kalman(rX);
			kry.kalman(rY);
			// PTU position
			kx.kalman(oyX); // take camera delay into account -> oyX instead of yX
			ky.kalman(oyY);			

		} // END Backup

		if(PTUctrl){ 
			// measure time for data collection:
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;   // us to ms
			// update old inputs:
			ouXrad = uXrad;
			ouYrad = uYrad;

			
			// LQR Controller:
			if(firstloop){ // for starting the control
				uXrad = K1*eX;
				uYrad = K1*eY;
				firstloop = false;
			}
			if(secondloop){  // for starting the control
				uXrad = K1*(krx.getestimate() - kx.getestimate()) + K2*krx.getstate2();
				uYrad = K1*(kry.getestimate() - ky.getestimate()) + K2*kry.getstate2();
				secondloop = false;
			}
			else{ // the actual controller
				uXrad = K1*(krx.getestimate() - kx.getestimate()) + K2*(krx.getostate2() - ouXrad) + krx.getstate2() - krx.getostate2() + ouXrad;
				uYrad = K1*(kry.getestimate() - ky.getestimate()) + K2*(kry.getostate2() - ouYrad) + kry.getstate2() - kry.getostate2() + ouYrad;
			}

/*
// Uncomment for feeding artificial reference signal
			//specific input signal:
			if(speccount<uspec.size()){
				uXrad = uspec[speccount];
				uYrad = 0;
			}
			else{
				uXrad = 0;
				uYrad = 0;
			}
			speccount++;
*/
			// Transform input from radians/second to positions/second
			uX = round(uXrad/pRes);
			uY = round(uYrad/tRes);

			// input limits:
			if(fabs(uX)<uXmin){
				if(fabs(pixelX-hWidth)>(double)tolerance*3){//
					uX = copysign(uXmin,uX);
				}
				else uX = 0;
			}
			else if(fabs(uX)>uXmax){
				uX = copysign(uXmax,uX);
			}
			if(fabs(uY)<uYmin){
				if(fabs(pixelY-hHeight)>(double)tolerance){//
					uY = copysign(uYmin,uY);
				}
			else uY = 0;
			}
			else if(fabs(uY)>uYmax){
				uY = copysign(uYmax,uY);
			}

			// input rate limit:
			if(fabs(uX-ouX)>deltaUXmax){
				if(uX>ouX) uX = ouX + deltaUXmax;
				else uX = ouX - deltaUXmax;
				// correct small velocities (otherwise speed would be increased again
				if(fabs(uX)<uXmin){
					if(fabs(pixelX-hWidth)>(double)tolerance*3){//
						uX = copysign(uXmin,ouX);
						if(fabs(uX-ouX)>deltaUXmax){
							if(fabs(ouX)>deltaUXmax) uX = copysign(uXmin,ouX);
							else if(fabs(ouX)<deltaUXmax) uX = -copysign(uXmin,ouX);
							else uX = 0;
						}
					}
					else uX = 0;
				}					
			}
			if(fabs(uY-ouY)>deltaUYmax){
				if(uY>ouY) uY = ouY + deltaUYmax;
				else uY = ouY - deltaUYmax;
				// correct small velocities (otherwise speed would be increased again
				if(fabs(uY)<uYmin){
					if(fabs(pixelY-hHeight)>(double)tolerance){//
						uY = copysign(uYmin,ouY);
						if(fabs(uY-ouY)>deltaUYmax){
							if(fabs(ouY)>deltaUYmax) uY = copysign(uYmin,ouY);
							else if(fabs(ouY)<deltaUYmax) uY = -copysign(uYmin,ouY);
							else uY = 0;
						}
					}
				else uY = 0;
				}
			}		
			// Avoid inputs going from too high to zero (see report...)
			if((uX== 0) && (abs(ouX)>94)) uX = copysign(uXmin,ouX);
			if((uY== 0) && (abs(ouY)>94)) uY = copysign(uYmin,ouY);


			// Uncomment one of the following two lines for examining single axis:
			//uY = 0;
			//uX = 0;
			
			// Adjust the input in radians again to the limited ptu input
			uXrad = round(uX)*pRes;
			uYrad = round(uY)*tRes;
			ouXrad = uXrad;
			ouYrad = uYrad;
			ptucmd.x = round(uX);
			ptucmd.y = round(uY);
			ptucmd.theta = 0;
			// Send the input to the PTU serial node:
			ptucmd_pub_.publish(ptucmd);
			// Collect the data:
			if(collectdata){
				timeVec.push_back(elapsedTime);
				exVec.push_back(eX);
				eyVec.push_back(eY);
				uxVec.push_back(uXrad);
				uyVec.push_back(uYrad);
				panposVec.push_back(yX);
				tiltposVec.push_back(yY);
				targetX.push_back(rX);
				targetY.push_back(rY);
				rxhatVec.push_back(krx.getestimate());
				rx1hatVec.push_back(krx.getstate1());
				rx2hatVec.push_back(krx.getstate2());
				ryhatVec.push_back(kry.getestimate());
				ry1hatVec.push_back(kry.getstate1());
				ry2hatVec.push_back(kry.getstate2());
				xhatVec.push_back(kx.getestimate());
				x1hatVec.push_back(kx.getstate1());
				x2hatVec.push_back(kx.getstate2());
				yhatVec.push_back(ky.getestimate());
				y1hatVec.push_back(ky.getstate1());
				y2hatVec.push_back(ky.getstate2());
			}
		// update the old inputs:
		ouX = uX;
		ouY = uY;
		}
		else{ // if PTUctrl is disabled
			uX = 0;
			uY = 0;
			// input rate limit:
			if(fabs(uX-ouX)>deltaUXmax){
				if(uX>ouX) uX = ouX + deltaUXmax;
				else uX = ouX - deltaUXmax;
				// correct small velocities (otherwise speed would be increased again
				if(fabs(uX)<uXmin){
					uX = 0;
				}
			}
			if(fabs(uY-ouY)>deltaUYmax){
				if(uY>ouY) uY = ouY + deltaUYmax;
				else uY = ouY - deltaUYmax;
				// correct small velocities (otherwise speed would be increased again
				if(fabs(uY)<uYmin){
					uY = 0;
				}
			}	

			ptucmd.x = uX;
			ptucmd.y = uY;
			ptucmd.theta = 0;
			// Send the "stopping" input to the PTU serial node:
			ptucmd_pub_.publish(ptucmd);
			// update old inputs:
			ouX = uX;
			ouY = uY;
			ouXrad = uX*pRes;
			ouYrad = uY*tRes;
		}
	}
	// END Pixel position callback
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//	PTU position callback
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// Receive the PTU positions:
	void ptuposCb(const geometry_msgs::Pose2D::ConstPtr& ptupos){
		if(errorarrived){ // Check if the pixelposition has arrived since last position callback
			// update measurments:
			oyX = yX;
			yX = ptupos->x * pRes;
			oyY = yY;
			yY = ptupos->y * tRes;
			ptuposarrived = true;
			errorarrived = false;
			// Collect the time when the position has arrived.
			gettimeofday(&t2, NULL);
			elapsedTime = (t2.tv_sec - t1.tv_sec)*1000.0;      // sec to ms
			elapsedTime += (t2.tv_usec - t1.tv_usec)/1000.0;   // us to ms
			if(collectdata){
				timePosVec.push_back(elapsedTime);
			}
		}
		if(writedata){
				// Write the collected data to a text file (CSV)
				writedata = false;
				std::stringstream ss;
				ss << "PTUExperiment_" <<std::setfill('0')<<std::setw(3)<<
					filenumber <<".txt"; // filenumber is updated, such that the text files are not overwritten.
				std::string filename = ss.str();
				myfile.open(filename.c_str());
				ROS_INFO("file opened %s \n",filename.c_str());
				myfile << "t, ex, ey, ux, uy, x, y, x_hat, x1_hat, x2_hat, y_hat, y1_hat, y2_hat,rx, ry, rx_hat, rx1_hat, rx2_hat, ry_hat, ry1_hat, ry2_hat, tp\n";
				for (int i = 0; i < timeVec.size(); i++) {
					myfile << std::setprecision(14) << timeVec[i] << "," <<
						std::setprecision(6) << exVec[i] << "," <<
						std::setprecision(6) << eyVec[i] << "," <<
						std::setprecision(6) << uxVec[i] << "," <<
						std::setprecision(6) << uyVec[i] << "," <<
						std::setprecision(6) << panposVec[i] << "," <<
						std::setprecision(6) << tiltposVec[i] << "," <<
						std::setprecision(6) << xhatVec[i] << "," <<
						std::setprecision(6) << x1hatVec[i] << "," <<
						std::setprecision(6) << x2hatVec[i] << "," <<
						std::setprecision(6) << yhatVec[i] << "," <<
						std::setprecision(6) << y1hatVec[i] << "," <<
						std::setprecision(6) << y2hatVec[i] << "," <<
						std::setprecision(6) << targetX[i] << "," <<
						std::setprecision(6) << targetY[i] << "," <<
						std::setprecision(6) << rxhatVec[i] << "," <<
						std::setprecision(6) << rx1hatVec[i] << "," <<
						std::setprecision(6) << rx2hatVec[i] << "," <<
						std::setprecision(6) << ryhatVec[i] << "," <<
						std::setprecision(6) << ry1hatVec[i] << "," <<
						std::setprecision(6) << ry2hatVec[i] << "," <<
						std::setprecision(14) << timePosVec[i] << "\n";
				}
				myfile.close();
				ROS_INFO("Data Written to File\n");
				filenumber++;
		}
	}
	// END PTU position callback
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//	Waitkey callback
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	// The waitkey is used for controlling this node:
	void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
		key = waitkey->data;
		switch(key){
			case -1:
				break;
			case 115: //s: start ptu control
				PTUctrl ^= true;
				// start time measurment:
				gettimeofday(&t1, NULL);
				// start collecting data:
				collectdata  = false;//^= true;
				// make sure the loop starts with a position measurement:
				errorarrived = true;
				ptuposarrived = false;
				// clear old data:
				if(collectdata){
					timeVec.clear();
					exVec.clear();
					eyVec.clear();
					uxVec.clear();
					uyVec.clear();
					panposVec.clear();
					tiltposVec.clear();
					targetX.clear();
					targetY.clear();
					xhatVec.clear();
					x1hatVec.clear();
					x2hatVec.clear();
					yhatVec.clear();
					y1hatVec.clear();
					y2hatVec.clear();
					rxhatVec.clear();
					rx1hatVec.clear();
					rx2hatVec.clear();
					ryhatVec.clear();
					ry1hatVec.clear();
					ry2hatVec.clear();
					timePosVec.clear();
				}
				eX = 0, eY = 0;
				firstloop = true, secondloop = true, thirdloop = true;
				if(PTUctrl) ROS_INFO("\n!!PTUctrl is ON!!\n");
				else ROS_INFO("\n!!PTUctrl is OFF!!\n");
				break;

			case 119: // w: write data to file
				writedata = true;
				break;
			case 114: // r: rehome PTU
				ROS_INFO("Rehome PTU\n");
				PTUctrl = false;
				firstloop = true;
				secondloop = true;
				collectdata = false;
				break;
		}
	}
	// END Waitkey Callback
	// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// shutdown node:
	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown\n");
		ros::shutdown();
	}
	// update the angle of view:
	void zoomlevelCb(const std_msgs::Int16::ConstPtr& zoomlevel){
		std::map<int,double>::iterator searchX= mapAngleOfViewX.find(zoomlevel->data);
		if( searchX != mapAngleOfViewX.end()){
			alphaX = searchX->second;
		}
		std::map<int,double>::iterator searchY = mapAngleOfViewY.find(zoomlevel->data);
		if( searchY != mapAngleOfViewY.end()){
			alphaY = searchY->second;
		}
	}

	// Adjust frame properties when grabbing frames at other resolution:
	void setframeprops(int width, int height){
		frWidth = width;
		frHeight = height;
		hWidth = frWidth/2;
		hHeight = frHeight/2;
	}

};
// END LQR Class
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	Main function
// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(int argc, char **argv){

	ros::init(argc,argv,"LQR_node");
	// construct object of LQR class:
	LQR cm;
	// Constantly call the callbacks:
	ros::spin();

	return 0;
}
