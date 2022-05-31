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
#include <mutex>
#include <unistd.h>

#include "serialCommArdRasp.hpp"
#include "myKalmanFilter.h"
#include "pacman_raspi.hpp"
#include "pacman_imu.hpp"

#include "global.h" // defines global variables

using namespace std;

int test_number = 2; // the variable is defined as global in global.h
mutex mx;
bool startCommunication = false;
serialCommArdRasp ser;
MyKalmanFilter filter;

int main()
{	
//serial.serLock = &mx;
	CAMERA camera = CAMERA();	
//camera.ser = &serial;
	
	float Rvalues[] = {0.9992, 0.0019, 0.0219, 0.0348};
	float Qvalues[] = { 0.000828343437286, 0.000828343437286, 0.000829718022605, 0.000828343437286 };// {0.999998969630621, 0.000828343437286, 0.000829718022605, 0.000828343437286 };// { 1, 0.000828343437285, 0.000829718022605, 0.000828343437285 };

	float **myQ = new float*[4];
	float **myR = new float*[4];

	for (int r = 0; r < 4; r++) {
		myQ[r] = new float[4];
		myR[r] = new float[4];
	}

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			if (r == c) {
				myR[r][c] = Rvalues[r];
				myQ[r][c] = Qvalues[r];
			} else {
				myR[r][c] = 0;
				myQ[r][c] = 0;
			}
		}
	}
	
	filter.setQ(myQ);
	filter.setR(myR);
	
	filter.serial = &ser;
	
	camera.filt = &filter;
	camera.camCommunicationON = &startCommunication;
	
	cout<<"inizializzando la seriale"<<endl;
	
	ser.init();
	ser.sendInitializingChar('q');
	
	cout<<"dormo 2 secondi"<<endl;
	sleep(2);
	
	cout<<"ora cominciano i thread"<<endl;
	
	thread tC(&CAMERA::Run, camera);
	thread tI(PhidgetRun, &filter, &startCommunication);
	
	ser.waitForStartingCommunication();
	cout<<"ora invio i dati"<<endl;
	startCommunication = true;
	
	
	if ( tI.joinable() ) {
		tI.join();
	}
	
	if ( tC.joinable() ) {
		tC.join();
	}
	
	cout<<"fine programma"<<endl;
	
	ser.sendCharToArduino(')');
	
	return 0;
}
