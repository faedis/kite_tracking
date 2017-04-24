#include <ros/ros.h>
#include <std_msgs/Int8.h>
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

//Specific input signal
std::vector<double> uspec;
//Specific reference signal
std::vector<double> rspec;
int speccount = 0;


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

void Kalman::kalman(double measurement){
	this -> ox1_hat = this -> x1_hat;
	this -> ox2_hat = this -> x2_hat;
	this -> x1_hat  = a11 * this -> ox1_hat + a12 * this -> ox2_hat + a13 * measurement;
	this -> x2_hat  = a21 * this -> ox1_hat + a22 * this -> ox2_hat + a23 * measurement;
	this -> x_hat   = a31 * this -> ox1_hat + a32 * measurement; 
}



class LQR{
	ros::NodeHandle nh_;
	ros::Subscriber pixelpos_sub_;
	ros::Subscriber comparedpixelpos_sub_;
	ros::Subscriber waitkey_sub_;
	ros::Subscriber shutdownkey_sub_;
	ros::Subscriber ptupos_sub_;
	ros::Publisher ptucmd_pub_;
	ros::Publisher targetpos_pub_;
private:
geometry_msgs::Pose2D ptucmd;
geometry_msgs::Pose2D targetpos;
// Kalman objects:
Kalman
	krx,
	kry,
	kx,
	ky;

std::ofstream myfile;
int filenumber = 0;
bool collectdata = false, writedata = false;
struct timeval t1, t2;
double elapsedTime;
bool errorarrived = false, ptuposarrived = false;
bool firstloop = true, secondloop = true, thirdloop = true;
bool PTUctrl = false;
double alphaX = 0.079555; // correction  by measurements 1.055
double alphaY = 0.044814; // correction  by measurements 1.021
double pRes = 92.5714 / 3600.0 * M_PI / 180.0;
double tRes = 46.2857 / 3600.0 * M_PI / 180.0;
double frWidth = 640;
double frHeight = 360;
double hWidth = frWidth/2;
double hHeight = frHeight/2;
int uXmax= 5000, uXmin = 60;		// pan input limits pos/second
int uYmax =5000, uYmin = 60;		// tilt input limits pos/second
int deltaUXmax  = 200; //= 400;				// pan input rate limit pos/Ts
int deltaUYmax = 400;				// tilt input rate limit pos/Ts
double tolerance = 6;				// pixel		(other approach:0.00244; // rad)

std::vector<double> timeVec;
std::vector<double> exVec;
std::vector<double> eyVec;
std::vector<double> uxVec;
std::vector<double> uyVec;
std::vector<double> rfxVec;
std::vector<double> rfyVec;
std::vector<double> yfxVec;
std::vector<double> yfyVec;
std::vector<double> panposVec;
std::vector<double> tiltposVec;
std::vector<double> targetX;
std::vector<double> targetY;
std::vector<double> efxVec;
// additional!!!
std::vector<double> timePosVec;
std::vector<double> xhatVec;
std::vector<double> x1hatVec;
std::vector<double> x2hatVec;
std::vector<double> rhatVec;
std::vector<double> r1hatVec;
std::vector<double> r2hatVec;
std::vector<double> tsizeVec;


//additional end!!!

int key = -1;
// Variables
double 
pixelX = 0,				// local error pan in pixel
pixelY = 0,				// local error tilt in pixel
tsize = 0,				// target size
ptuX = 0,				// ptu pan rad
ptuY = 0,				// ptu tilt rad
yX = 0,					// ptu pan rad
oyX = 0,
yY = 0,					// ptu tilt rad
oyY = 0,
rX = 0,					// target pan rad
orX = 0,		
rY = 0,					// target tilt rad
orY = 0,
eX = 0,					// local error pan rad
eY = 0,					// local error tilt rad
uXrad = 0,				// input pan in rad/second
ouXrad = 0,
uYrad = 0,				// input tilt in rad/second
ouYrad = 0,
uX = 0,					// input pan in ptu positions/second
uY = 0,					// input tilt in ptu positions/second
ouX = 0,				// old input tilt in ptu positions/second
ouY = 0;				// old input tilt in ptu positions/second

// Constants
double
K1 = 3.1448,//3.1448,//2.4978,//			// LQR gain 1
K2 = 0.6103,//0.6103,//0.4593,//			// LQR gain 2
Ts = 0.03333;			// sampling time

// kalman filter parameters
// reference
double
Mx1 = 0.8181,//0.8181,			// current values corresond to Q = 100, R = 0.00007
Mx2 = 16.9903,//16.9903,
My1 = 0.6867,//0.6867,			// current values corresond to Q = 100, R = 0.0003
My2 = 10.7716;//10.7716, 





public:
	LQR(){
//		pixelpos_sub_ = nh_.subscribe("pixelpos",1,&LQR::pixelposCb,this);
		comparedpixelpos_sub_ = nh_.subscribe("comparedpixelpos",1,&LQR::comparedpixelposCb,this);
		waitkey_sub_  = nh_.subscribe("waitkey",1,&LQR::waitkeyCb,this);
		shutdownkey_sub_ = nh_.subscribe("shutdownkey",1,&LQR::shutdownCb,this);
		ptupos_sub_ = nh_.subscribe("ptupos",1,&LQR::ptuposCb,this);
		ptucmd_pub_ = nh_.advertise<geometry_msgs::Pose2D>("ptucmd",1);
		targetpos_pub_ = nh_.advertise<geometry_msgs::Pose2D>("targetpos",1);
		kx.kalmanInit(Ts, Mx1, Mx2);
		ky.kalmanInit(Ts, My1, My2);
		krx.kalmanInit(Ts, Mx1, Mx2);
		kry.kalmanInit(Ts,My1, My2);
	}
	~LQR(){
	}
//	void pixelposCb(const geometry_msgs::Pose2D::ConstPtr& pixelpos){
	void comparedpixelposCb(const geometry_msgs::Pose2D::ConstPtr& comparedpixelpos){
		if(ptuposarrived){
			pixelX = comparedpixelpos->x;
			pixelY = comparedpixelpos->y;
			tsize = comparedpixelpos->theta;
			if(pixelX>=0){
			// assign errors
			eX = -alphaX * (double)(pixelX - hWidth) / (double)hWidth;
			eY = -alphaY * (double)(pixelY - hHeight) / (double)hHeight;
			}
			else{
				eX = 0.8*eX;
				eY = 0.8*eY;
			}
		


			// update and publish target position if all information has arrvied
			errorarrived = true;
			if(ptuposarrived){
				orX = rX;
//				rX = yX + eX;
				rX = oyX+eX; // take camera delay into account
				orY = rY;
//				rY = yY + eY;
				rY = oyY+eY;
				errorarrived = false, ptuposarrived = false;
				targetpos.x = rX;
				targetpos.y = rY;
				targetpos.theta = 0;
				targetpos_pub_.publish(targetpos);
				//ROS_INFO("e, r, x 	%f	%f	%f",eX,rX,oyX);
/*	
				if(PTUctrl){
				// Specific Reference Signal:
					if(speccount<rspec.size()){
						rX = rspec[speccount];
						rY = 0;
						eX = rX - yX;
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
/////////////////////////////////////////////////////////////////////////
				// Kalman Filter +++++++++++++++++++++++++++++++++++++++

				// update:
				// Reference
				krx.kalman(rX);
				kry.kalman(rY);
				// PTU position
				kx.kalman(oyX); // take camera delay into account -> oyX instead of yX
				ky.kalman(oyY);
/////////////////////////////////////////////////////////////////////////
//				ROS_INFO("r,rx, rx2 		%f	%f	%f",rX, krx.getestimate(), krx.getstate2());

			}
			if(PTUctrl){

				gettimeofday(&t2, NULL);
				elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
				elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms
				// update old inputs
				ouXrad = uXrad;
				ouYrad = uYrad;


 				// LQR Controller

				// Controller input taking camera delay into account
				if(firstloop){
					uXrad = K1*eX;
					uYrad = K1*eY;
					firstloop = false;
				}
				if(secondloop){
					uXrad = K1*(krx.getestimate() - kx.getestimate()) + K2*krx.getstate2();
					uYrad = K1*(kry.getestimate() - ky.getestimate()) + K2*kry.getstate2();
					secondloop = false;
				}
				else{
					uXrad = K1*(krx.getestimate() - kx.getestimate()) + K2*(krx.getostate2() - ouXrad) + krx.getstate2() - krx.getostate2() + ouXrad;
					uYrad = K1*(kry.getestimate() - ky.getestimate()) + K2*(kry.getostate2() - ouYrad) + kry.getstate2() - kry.getostate2() + ouYrad;
				}

/*
				//specific input signal
		
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

				uX = round(uXrad/pRes);
				uY = round(uYrad/tRes);
		

				// input rate limit:
				if(abs(uX-ouX)>deltaUXmax){
					if(uX>ouX) uX = ouX + deltaUXmax;
					else uX = ouX - deltaUXmax;
					// correct small velocities (otherwise speed would be increased again
					if(abs(uX)<uXmin){
						if(abs(pixelX-hWidth)>tolerance){//
							uX = copysign(uXmin,ouX);
						}
						else uX = 0;
					}					
				}
				if(abs(uY-ouY)>deltaUYmax){
					if(uY>ouY) uY = ouY + deltaUYmax;
					else uY = ouY - deltaUYmax;
					// correct small velocities (otherwise speed would be increased again
					if(abs(uY)<uYmin){
						if(abs(pixelY-hHeight)>tolerance){//
							uY = copysign(uYmin,ouY);
						}
					else uY = 0;
					}
				}		

				// Additioinal input rate limit:
/*				if(abs(uX)<abs(ouX)){
					if(abs(uX-ouX)<59){
						uX = ouX;				
					}
				}
*/
				// input limits:
				if(abs(uX)<uXmin){
					if(abs(pixelX-hWidth)>tolerance){//
						uX = copysign(uXmin,uX);
					}
					else uX = 0;
				}
				else if(abs(uX)>uXmax){
					uX = copysign(uXmax,uX);
				}
				if(abs(uY)<uYmin){
					if(abs(pixelY-hHeight)>tolerance){//
						uY = copysign(uYmin,uY);
					}
				else uY = 0;
				}
				else if(abs(uY)>uYmax){
					uY = copysign(uYmax,uY);
				}

				// Examine only pan
				uY = 0;

				uXrad = round(uX)*pRes;
				uYrad = round(uY)*tRes;
				ouXrad = uXrad;
				ouYrad = uYrad;

				ptucmd.x = round(uX); //floor
				ptucmd.y = round(uY); //floor
				ptucmd.theta = 0;
				ptucmd_pub_.publish(ptucmd);
//				ROS_INFO("cmd");
				if(collectdata){
					timeVec.push_back(elapsedTime);
					exVec.push_back(eX);
//					eyVec.push_back(eY);
					uxVec.push_back(uXrad);//uxVec.push_back(uXrad);
//					uyVec.push_back(uYrad);
//					yfxVec.push_back(kx.getestimate());
//					yfyVec.push_back(ky.getestimate());
					panposVec.push_back(yX);
//					tiltposVec.push_back(yY);
					targetX.push_back(rX);
//					targetY.push_back(rY);
//					rfxVec.push_back(krx.getestimate());
//					rfyVec.push_back(kry.getestimate);
					// additional
					rhatVec.push_back(krx.getestimate());
					xhatVec.push_back(kx.getestimate());
					r1hatVec.push_back(krx.getstate1());
					r2hatVec.push_back(krx.getstate2());
					x1hatVec.push_back(kx.getstate1());
					x2hatVec.push_back(kx.getstate2());
//					tsizeVec.push_back(tsize);
				}

			ouX = uX;
			ouY = uY;
			}
			else{
				// input rate limit:
				if(abs(uXrad)>deltaUXmax){
					if(uXrad>0) uXrad = ouXrad - deltaUXmax;
					else uXrad = ouXrad + deltaUXmax;
				}
				else uXrad = 0;
				if(abs(uXrad)<uXmin) uXrad =0;
				if(abs(uYrad)>deltaUYmax){
					if(uYrad>0) uYrad = ouYrad - deltaUYmax;
					else uYrad = ouYrad + deltaUYmax;
				}
				else uYrad = 0;
				if(abs(uYrad)<uYmin) uYrad = 0;
				uX = uXrad/pRes;
				uY = uYrad/tRes;
				// send ptucmd
				ptucmd.x = round(uX);
				ptucmd.y = round(uY);
				ptucmd.theta = 0;
				ptucmd_pub_.publish(ptucmd);
			}
		}
	}
	void ptuposCb(const geometry_msgs::Pose2D::ConstPtr& ptupos){
		//update states yx, yy and old ones
		oyX = yX;
		yX = ptupos->x * pRes;
		oyY = yY;
		yY = ptupos->y * tRes;
		ptuposarrived = true;
//		ROS_INFO("POSITION ARRIVED");
//additional!!!
		gettimeofday(&t2, NULL);
		elapsedTime = (t2.tv_sec - t1.tv_sec)*1000;      // sec to ms
		elapsedTime += (t2.tv_usec - t1.tv_usec)/1000;   // us to ms

		if(collectdata){
			timePosVec.push_back(elapsedTime);
		}
		if(writedata){
				writedata = false;
				std::stringstream ss;
				ss << "ManipSinusoidalCheckOsci0419_" <<std::setfill('0')<<std::setw(3)<<
					filenumber <<".txt";
				std::string filename = ss.str();
				myfile.open(filename.c_str());
				ROS_INFO("file opened %s \n",filename.c_str());
/*				myfile << "t, ex, ey, ux, uy, xy, yy, yfx, yfy, rx, ry, rfx, rfy, tp\n";
				for (int i = 0; i < timeVec.size(); i++) {
					myfile << std::setprecision(14) << timeVec[i] << "," <<
						std::setprecision(8) << exVec[i] << "," <<
						std::setprecision(8) << eyVec[i] << "," <<
						std::setprecision(8) << uxVec[i] << "," <<
						std::setprecision(8) << uyVec[i] << "," <<
						std::setprecision(8) << panposVec[i] << "," <<
						std::setprecision(8) << tiltposVec[i] << "," <<
						std::setprecision(8) << yfxVec[i] << "," <<
						std::setprecision(8) << yfyVec[i] << "," <<
						std::setprecision(8) << targetX[i] << "," <<
						std::setprecision(8) << targetY[i] << "," << //"\n";
						std::setprecision(8) << rfxVec[i] << "," <<
						std::setprecision(8) << rfyVec[i] << "," <<
						std::setprecision(14) << timePosVec[i] << "\n";
				}
*/
				myfile << "t, ex, ux, yx, x_hat, x1_hat, x2_hat, r, r_hat, r1_hat, r2_hat, tsize, tp\n";
				for (int i = 0; i < timeVec.size(); i++) {
					myfile << std::setprecision(14) << timeVec[i] << "," <<
//						std::setprecision(8) << exVec[i] << "," <<
						std::setprecision(8) << uxVec[i] << "," <<
						std::setprecision(8) << panposVec[i] << "," <<
						std::setprecision(8) << xhatVec[i] << "," <<
						std::setprecision(8) << x1hatVec[i] << "," <<
						std::setprecision(8) << x2hatVec[i] << "," <<
						std::setprecision(8) << targetX[i] << "," <<
						std::setprecision(8) << rhatVec[i] << "," <<
						std::setprecision(8) << r1hatVec[i] << "," <<
						std::setprecision(8) << r2hatVec[i] << "," <<
//						std::setprecision(8) << tsizeVec[i] << "," <<
						std::setprecision(14) << timePosVec[i] << "\n";
				}
				myfile.close();
				ROS_INFO("Data Written to File\n");
				filenumber++;
		}
	}

/*	// map zoomlevel to alpha
	void angleOfViewCb(const std_msgs::Int8::ConstPtr& zoomlevel){
		alphaX = PanAngleMap.find(zoomlevel->data)->second;
		alphaY = TiltAngleMap.find(zoomlevel->data)->second;

	}
*/
	void waitkeyCb(const std_msgs::Int8::ConstPtr& waitkey){
		//ROS_INFO("waitkeyarrived\n");
		key = waitkey->data;
		switch(key){
			case -1:
				break;
			case 115: //s: start ptu control
				PTUctrl ^= true;
				// initiatlize Kalman filter
//				kx.kalmanInit(Ts, Mx1, Mx2);
//				ky.kalmanInit(Ts, My1, My2);
//				krx.kalmanInit(Ts, Mx1, Mx2);
//				kry.kalmanInit(Ts,My1, My2);
				gettimeofday(&t1, NULL);

				collectdata ^= true;
				errorarrived = false;
				ptuposarrived = false;
				if(collectdata){
					timeVec.clear();
					exVec.clear();
					eyVec.clear();
					uxVec.clear();
					uyVec.clear();
					panposVec.clear();
					tiltposVec.clear();
					yfxVec.clear();
					yfyVec.clear();
					targetX.clear();
					targetY.clear();
					rfxVec.clear();
					rfyVec.clear();
					// additional!!
					timePosVec.clear();
					rhatVec.clear();
					r1hatVec.clear();
					r2hatVec.clear();
					xhatVec.clear();
					x1hatVec.clear();
					x2hatVec.clear();
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
				//speccount = 0;
				PTUctrl = false;
				firstloop = true;
				collectdata = false;
				break;
		}
	}

	void shutdownCb(const std_msgs::Bool::ConstPtr& shutdownkey){
		ROS_INFO("Shutdown\n");
		ros::shutdown();
	}

	void setframeprops(int width, int height){
		frWidth = width;
		frHeight = height;
		hWidth = frWidth/2;
		hHeight = frHeight/2;
	}

};

int main(int argc, char **argv){

	// generate specific input signal
	double a = 0.25;
	for (int i = 0; i<10; i++){
		uspec.push_back(0.0);
	}

	for (int i = 0; i<960; i++){
		uspec.push_back(a*cos((double)i*2.0*M_PI/480.0));
	}
	for (int i = 0; i <30; i++){
		uspec.push_back(0);
	}
	uspec[35] = uspec[34] + 0.01;
//	uspec[36] = 0;
//	uspec[37] = 0;
//	uspec[38] = 0;
	uspec[300] = uspec[299]-0.01;
//	uspec[281] = 0.0;


/*
	std::string line;
	std::string::size_type sz;     // alias of size_t
	std::ifstream inputfile ("/home/guetgf/catkin_ws/src/subscribetoimage/src/lollyInputOsci.txt");
	if (inputfile.is_open())
	{
	while ( std::getline (inputfile,line) )
	{
	  uspec.push_back(std::stod(line,&sz));
	}
	inputfile.close();
	}
	else{
		std::cout << "Unable to open file";
		return 0;
	}
*/

/*
	for (int i = 0; i<50; i++){
		rspec.push_back(0.0);
	}
	for(int i = 0; i<40;i++){
		rspec.push_back((double)i*0.01);
	}	

	rspec.push_back(0.39+0.007);
	rspec.push_back(0.39+0.003);
	rspec.push_back(0.39+0.011);
	rspec.push_back(0.39+0.011);
	for(int i = 0; i<100;i++){
		rspec.push_back(0.401);
	}	
*/

	ros::init(argc,argv,"LQR_node");
	LQR cm;

	if(argc == 3){
		int frWidth = atoi(argv[1]);
		int frHeight = atoi(argv[2]);
		cm.setframeprops(frWidth,frHeight);
	std::cout << "\nframe width 	" << frWidth
	<< "\nframe height	" << frHeight << "\n"; 
	}
	else{
		std::cout<< "Default frame property values used [640x360].\n You can pass the arguments using: [frame width]  [frame height] in command line\n";
	}



	ros::spin();
	return 0;
}
