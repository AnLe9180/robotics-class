/*
TestProgram.cpp by Jason Morris and Mike McGrath
This simple program shows what is needed for a program to control the Aria based robot.
*/
//C++ class to allow communication of the program to a user
#include "../00_nav_library/nav_aria_iface.h"
#include "../00_nav_library/nav_random.h"
#include "../00_nav_library/nav_map.h"
#include "../00_nav_library/nav_robot2map.h"
#include "../00_nav_library/nav_field_potential.h"
//#include "nav_robot2lines.h"
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <iomanip>
#include <conio.h>
#include <sstream>
#include <complex>  

#include <math.h>       /* atan2 */
#include <stdlib.h>     /* abs */
using namespace std;
//This class is what allows a robot
//object to be created to communicate to an Aria based robot.




void Follow_Circle(NAV_Robot & R) {

	char file[] = "posePioneer.csv";
	std::ofstream data;
	data.open(file);

	char input;
	int k = 0;
	const double tf = 60;
	const double T = 50;
	double time = 0;
	const double Rad = 1;
	double gamma, delta;
	double rho, xdt, ydt, thetadt, xd_dott, yd_dott, vdt, x = 2, y = 1, theta = M_PI / 2, tmp, k1, k2, k3, v, w, e1, e2, e3, xd_dot_dott, yd_dot_dot;

	//do{
	while (time<tf) {

		xdt = 0;
		ydt = 0;


		x = R.getX() / 1000;
		y = R.getY() / 1000;
		theta = R.getTh()*(M_PI / 180);
		if (theta > 2 * M_PI)
		{
			theta = theta - (2 * M_PI);
		}

		if (theta < 0)
		{
			theta = theta + (2 * M_PI);
		}
		cout << "time: " << time << endl;
		rho = sqrt(y*y+x*x);
		gamma = atan2(y,x) - theta + M_PI;
		delta = atan2(y,x) + theta;



		//print error
		k1 = .4;
		k2 = .5;
		k3 = .4;






		v = k1*rho*cos(gamma);
		w = k2*gamma+k1*(sin(gamma)/gamma)*(cos(gamma))*(gamma+k3*delta);

		//R.unlock();
		//cout << w << " " << v << endl;
		R.lock();
		//R.doRotateAngle(w);
		//R.doDriveSpeed(v);
		R.doDriveSpeed(v * 1000);
		R.doRotateSpeed(w*(180 / M_PI));
		R.unlock();
		R.Sleep(T);


		data << time << " " << x << " " << y << " " << theta << " " << v << " " << w << endl;


		//R.lock();
		k += 1;
		time = k*T / 1000;
		//} while (time<tf);
	}

	//R.doDriveSpeed(0);
	//R.unlock();
	return;
}



int main() {

	/* We create an object R of class NAV_Robot that will be used to control the robot
	the robot is in a local area network and will be connected through the IP
	192.168.4.4 with port 8101 */
	//NAV_Robot R("localhost", 8101);
	NAV_Robot R("COM9");

	/////////////	NAV_Robot R("localhost", 8101); //This Creates the object R that will be used to

	//control the robot in this case it will be connected
	//to the local port 8101 which MobileSim defaults to
	//listen on.
	/* You may want to connect the robot through COMx, where x is the COM port number
	when the robot is connected using a USB to serial cable*/
	////////////NAV_Robot R("COM1",8101);
	//This Creates the object R that will be used to
	//control the robot in this case it will be connected
	//to the COM1 port of your computer.
	if (!R.is_connected()) {
		// This checks if the robot is connected and returns a bool value
		std::cout << "Error in connecting" << std::endl;
		return(0);
	}


	R.moveCoords(ArPose(2000, 1000, 90));//This sets the robot to be at 0
	Follow_Circle(R);

	return(0);

}