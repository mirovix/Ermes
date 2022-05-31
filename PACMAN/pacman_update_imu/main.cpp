/*
 * main.cpp
 * project: pacman
 * 
 * the software handles the navigation tasks for the PACMAN experiment
 * 
 * developed by: Mattia Mazzucato
 * email: matt.mazzucato@gmail.com
 *
 */

#include <thread>
// #include <mutex>
// #include <unistd.h>

// #include "serialCommArdRasp.hpp"
// #include "myKalmanFilter.h"
// #include "pacman_raspi.hpp"
#include "pacman_imu.hpp"

// #include "global.h" // defines global variables

using namespace std;

// int test_number = 2; // the variable is defined as global in global.h
// mutex mx;
// bool startCommunication = false;
// serialCommArdRasp ser;
// MyKalmanFilter filter;

int main()
{	
	
	// thread tI(PhidgetRun, &filter, &startCommunication);
	thread tI(PhidgetRun);
	
	// ser.waitForStartingCommunication();
	
	if ( tI.joinable() ) {
		tI.join();
	}
		
	printf("fine programma\n\n");
		
	return 0;
}
