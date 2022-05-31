#ifndef PACMAN_SERIAL_
#define PACMAN_SERIAL_

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringSerial.h>
//#include "global.h"
#include <mutex>

using namespace std;



class serialCommArdRasp {
	public:
		serialCommArdRasp();
		serialCommArdRasp(char * usbDevice);
		serialCommArdRasp(unsigned int baudRate);
		serialCommArdRasp(char * usbDevice, unsigned int baudRate);

		bool init();
		bool init(char * usbDevice, unsigned int baudRate);
		void sendDataToArduino(float *numberToSend, int length);
		void sendCharToArduino(char charToSend);
		
		void sendInitializingChar(char a);
		void waitForStartingCommunication();
		bool isCommunicationOpen();
		
		//mutex *serLock;

	private:
		//static bool serialAvailable = true;
		bool communicationOpen = false;
		bool communicationInitialized = false;

		unsigned int BAUDRATE = 115200;		// const(?) if so, delete secondary constructors
		char * USB_DEVICE = "/dev/ttyS0";	// const(?) if so, delete secondary constructors
		int communication;

		
		void float2Bytes(unsigned char* floatBytes, float number);
};

#include "global.h"

#endif //PACMAN_SERIAL_
