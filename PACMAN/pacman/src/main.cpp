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

#include "pacman_raspi.hpp"
#include "pacman_imu.hpp"
#include <thread>
#include <mutex>
#include "global.h" // defines global variables

using namespace std;

int test_number = 2; // the variable is defined as global in global.h
mutex mx;

// todo handle serial communication with mutex 

int main()
{	
	
	CAMERA camera;	
	
	thread tC(&CAMERA::Run, camera);
	thread tI(PhidgetRun);
	
	if ( tI.joinable() ){
		tI.join();
	}
	
	if ( tC.joinable() )	{
		tC.join();
	}
	
	
	return 0;
}
