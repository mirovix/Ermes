/* pacman_imu.hpp
 * 
 * the library handles the acquisition of the phidget imu mounted on the 
 * Pacman experiment
 * 
 * library modified to exclude the magnetometer from the acquisition
 * 
 * developed by: Mattia Mazzucato
 * email: matt.mazzucato@gmail.com
 * 
 */

#include <stdio.h>
#include <stdlib.h>

#include <string>

#include <phidget22.h>
#include "global.h"

#ifndef _WIN32
#include <unistd.h>
#else
#include <Windows.h>
#endif

//#define PRINT_DBG_MSGS
//#define ENABLE_PHIDGET_LOG_INFO

#define FN_SIZE 80 // length of file name string

using namespace std;

// creo canali per l'acquisizione
PhidgetAccelerometerHandle 	chA;
PhidgetGyroscopeHandle		chG;

PhidgetReturnCode 	res;
const char*		errs;

double timeA, timeG;
double res_vec[7];

FILE *file;
char rslt_file_name[FN_SIZE];

static void CCONV ssleep(int);
const string currentDateTime(void);

uint32_t dt_sampling = 12; // intervallo campionamento

MyKalmanFilter *filt;
bool *imuCommunicationON;
int p = 0;


// ====================================================================================
static void CCONV onAttachHandler(PhidgetHandle phid, void *ctx) {
	PhidgetReturnCode res;
	int hubPort;
	int channel;
	int serial;

	res = Phidget_getDeviceSerialNumber(phid, &serial);
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to get device serial number\n");
		return;
	}

	res = Phidget_getChannel(phid, &channel);
		if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to get channel number\n");
		return;
	}

	res = Phidget_getHubPort(phid, &hubPort);
		if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to get hub port\n");
		hubPort = -1;
	}

	#ifdef PRINT_DBG_MSGS
	if (hubPort == -1)
		printf("channel %d on device %d attached\n", channel, serial);
	else
		printf("channel %d on device %d hub port %d attached\n", channel, serial, hubPort);
	#endif

	// set time interval [ms] between data acquisition
	res = PhidgetAccelerometer_setDataInterval((PhidgetAccelerometerHandle)phid, dt_sampling); // sono accettati solo multipli di 4 ms
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed set accelerometer dataInterval\n");
		hubPort = -1;
	}


	res = PhidgetGyroscope_setDataInterval((PhidgetGyroscopeHandle)phid, dt_sampling); // sono accettati solo multipli di 4 ms
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed set gyro dataInterval\n");
		hubPort = -1;
	}
	
	// generate result filename according to test number
	snprintf(rslt_file_name, FN_SIZE-1, "../results/TestIMU_%d__%s.dat", test_number ,currentDateTime().c_str());
	file = fopen(rslt_file_name,"w");
	
	if (file == NULL)
	{
		printf("Error opening file!\n");
		exit(1);
	}


	/*
	* Enable logging to stdout
	*/
	#ifdef ENABLE_PHIDGET_LOG_INFO
		PhidgetLog_enable(PHIDGET_LOG_INFO, NULL);
	#endif // ENABLE_PHIDGET_LOG_INFO

}


// ====================================================================================
const string currentDateTime(void)
{
	time_t     now = time(0);
	struct tm tstruct;
	char      buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%y%m%d_%H%M%S", &tstruct);

	return buf;
}


// ====================================================================================
static void CCONV onDetachHandler(PhidgetHandle phid, void *ctx) {
	PhidgetReturnCode res;
	int hubPort;
	int channel;
	int serial;

	res = Phidget_getDeviceSerialNumber(phid, &serial);
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to get device serial number\n");
		return;
	}

	res = Phidget_getChannel(phid, &channel);
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to get channel number\n");
		return;
	}

	res = Phidget_getHubPort(phid, &hubPort);
	if (res != EPHIDGET_OK)
		hubPort = -1;

	#ifdef PRINT_DBG_MSGS
	if (hubPort != -1)
		printf("channel %d on device %d detached\n", channel, serial);
	else
		printf("channel %d on device %d hub port %d detached\n", channel, hubPort, serial);
	#endif

}


// ====================================================================================
static void CCONV errorHandler(PhidgetHandle phid, void *ctx, 
						Phidget_ErrorEventCode errorCode, const char *errorString) 
{
	fprintf(stderr, "Error: %s (%d)\n", errorString, errorCode);
}


// =============================================================================================
void writeVecToFile()
{
	fprintf(file, "%.0f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\t%6.4f\n", res_vec[0],
			res_vec[1], res_vec[2], res_vec[3], res_vec[4], res_vec[5], res_vec[6]
			);
}


// ====================================================================================
static void CCONV onAccelerationChange(PhidgetAccelerometerHandle chA, 
						void *ctx, 
						const double* acceleration, 
						double timestamp) 
{
	printf("imu %d\n", p++);
	#ifdef PRINT_DBG_MSGS
		printf("Timestamp: %.0f\n", timestamp);
		printf("Acceleration Changed:%7.3f%8.3f%8.3f\n", acceleration[0], acceleration[1], acceleration[2]);
		printf("\n");
	# endif
	
	timeA = timestamp;
	
	res_vec[0] = timeA;
	res_vec[1] = acceleration[0];
	res_vec[2] = acceleration[1];
	res_vec[3] = acceleration[2];

	if (timeA == timeG)
		writeVecToFile();

}


// =============================================================================================
static void CCONV onAngularRateUpdate(PhidgetGyroscopeHandle ch,
						void *ctx,
						const double* angularRate,
						double timestamp)
{
	#ifdef PRINT_DBG_MSGS
		printf("Timestamp: %.0f\n", timestamp);
		printf("\t\t\tAngular velocity Changed:%7.3f%8.3f%8.3f\n", angularRate[0], angularRate[1], angularRate[2]);
		printf("\n");
	# endif

	timeG = timestamp;

	res_vec[0] = timeG;
	res_vec[4] = angularRate[0];
	res_vec[5] = angularRate[1];
	res_vec[6] = angularRate[2];

	// write data to results file
	if (timeA == timeG)
		writeVecToFile();

	
	if(*imuCommunicationON) {
	 	float angleVels[3];
	 	for(int i = 0; i<3; i++) {
	 		angleVels[i] = angularRate[i]*3.14/180.0;
	 	}
	 	//printf("%.0f\tprima", timestamp);
	 	filt->timeUpdate(angleVels, 0.012);
	 	//printf("\tdopo\n");
	}


}



// =============================================================================================
static void CCONV PhidgetRun(MyKalmanFilter *filter, bool *commStarter){
		
	filt = filter;
	imuCommunicationON = commStarter;
	
	// CREATE ===============================================================
	res = PhidgetAccelerometer_create(&chA);
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to create accelerometer channel\n");
		exit(1);
	}

	res = PhidgetGyroscope_create(&chG);
	if (res != EPHIDGET_OK) {
		fprintf(stderr, "failed to create gyroscope channel\n");
		exit(1);
	}



	// ON ATTACH ===============================================================
	res = Phidget_setOnAttachHandler( (PhidgetHandle)chA, onAttachHandler, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set  accelerometer onAttachHandler: %s\n", errs);
		goto done;
	}

	res = Phidget_setOnAttachHandler( (PhidgetHandle)chG, onAttachHandler, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set  gyroscope onAttachHandler: %s\n", errs);
		goto done;
	}


	// ON ERROR ===============================================================
	res = Phidget_setOnErrorHandler( (PhidgetHandle)chA, errorHandler, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set  accelerometer onErrorHandler: %s\n", errs);
		goto done;
	}

	res = Phidget_setOnErrorHandler( (PhidgetHandle)chG, errorHandler, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set  gyroscope onErrorHandler: %s\n", errs);
		goto done;
	}


	// ON EVENT ===============================================================
	res = PhidgetAccelerometer_setOnAccelerationChangeHandler(chA, onAccelerationChange, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set onAccelerationChange: %s\n", errs);
		goto done;
	}

	res = PhidgetGyroscope_setOnAngularRateUpdateHandler(chG, onAngularRateUpdate, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set onAngularRateUpdate: %s\n", errs);
		goto done;
	}


	// ON DETACH ===============================================================
	res = Phidget_setOnDetachHandler( (PhidgetHandle)chA, onDetachHandler, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set accelerometer onDetachHandler: %s\n", errs);
		goto done;
	}

	res = Phidget_setOnDetachHandler( (PhidgetHandle)chG, onDetachHandler, NULL);
	if (res != EPHIDGET_OK) {
		Phidget_getErrorDescription(res, &errs);
		fprintf(stderr, "failed to set accelerometer onDetachHandler: %s\n", errs);
		goto done;
	}

	// ===============================================================
	

	/*
	* Open the channel synchronously: waiting a maximum of 5 seconds.
	*/
	res = Phidget_openWaitForAttachment((PhidgetHandle)chA, 5000);
	if (res != EPHIDGET_OK) {
		if (res == EPHIDGET_TIMEOUT) {
			printf("Channel did not attach after 5 seconds: please check that the device is attached\n");
		} else {
			Phidget_getErrorDescription(res, &errs);
			fprintf(stderr, "failed to open channel:%s\n", errs);
		}
		goto done;
	}


	res = Phidget_openWaitForAttachment((PhidgetHandle)chG, 5000);
	if (res != EPHIDGET_OK) {
		if (res == EPHIDGET_TIMEOUT) {
			printf("Channel did not attach after 5 seconds: please check that the device is attached\n");
		} else {
			Phidget_getErrorDescription(res, &errs);
			fprintf(stderr, "failed to open channel:%s\n", errs);
		}
		goto done;
	}



	#ifdef PRINT_DBG_MSGS
	printf("Gathering data for 30 seconds...\n");
	#endif
	
	ssleep(30);

	done:

	// Phidget_close((PhidgetHandle)ch);
	PhidgetAccelerometer_delete(&chA);
	PhidgetGyroscope_delete(&chG);


	// close results file
	fclose(file);

	
	printf("Imu acquisition concluded\n");

	//exit(res);
	return;
}


static void CCONV ssleep(int tm) {
	#ifdef _WIN32
	Sleep(tm * 1000);
	#else
	sleep(tm);
	#endif
}
