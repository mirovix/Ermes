#include "serialCommArdRasp.hpp"
#include <iostream>

#define SER_DEBUG
//#define SER_DEBUG_EXTREME
/*--------------------------------------------------------- CONSTRUCTORS ---------------------------------------------------------*/

serialCommArdRasp::serialCommArdRasp() { // uses the default usb device and baud rate

}

serialCommArdRasp::serialCommArdRasp(char * usbDevice) { // uses the given usb device and the default baud rate 
	this->USB_DEVICE = usbDevice;
}

serialCommArdRasp::serialCommArdRasp(unsigned int baudRate) { // uses the default usb device and the given baud rate
	this->BAUDRATE = baudRate;
}

serialCommArdRasp::serialCommArdRasp(char * usbDevice, unsigned int baudRate) { // uses the given usb device and baud rate
	this->USB_DEVICE = usbDevice;
	this->BAUDRATE = baudRate;
}

/*--------------------------------------------------------------------------------------------------------------------------------*/

void serialCommArdRasp::sendInitializingChar(char a){
	//serLock->lock();
	
	serialPutchar(communication, a);
	while (serialDataAvail(communication) <= 0) { /*wait for the arduino answer*/ }
	
	char received = serialGetchar(communication);
	while (received != 107) { // received != 'k'
		while (serialDataAvail(communication) <= 0) { /*wait for the arduino answer*/ }
		received = serialGetchar(communication);
	}
	#ifdef SER_DEBUG
		printf("comunicazione inizializzata.\n");
	#endif
	
	
	communicationInitialized = true;
	
	//serLock->unlock();
}

void serialCommArdRasp::waitForStartingCommunication() {
	while (serialDataAvail(communication) <= 0) { /*wait for the arduino answer*/ }
	
	char received = serialGetchar(communication);
	if (received != 40) { // received != '('
		while (true) { printf("comunicazione inizializzata.\n"); } // <--------------------- DA CAMBIAREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
	}
}

bool serialCommArdRasp::init(char * usbDevice, unsigned int baudRate) {
	if ((communication = serialOpen(usbDevice, baudRate)) < 0)
	{
		#ifdef SER_DEBUG
			fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		#endif
		return false;
	}
	communicationOpen = true;
	return true;
}

bool serialCommArdRasp::init() {
	return init(this->USB_DEVICE, this->BAUDRATE);
}

bool serialCommArdRasp::isCommunicationOpen(){
	return communicationOpen;
}

void serialCommArdRasp::float2Bytes(unsigned char* floatBytes, float number) {
	union {
		float floatVar;
		unsigned char floatArray[4];
	} var;
	var.floatVar = number;
	memcpy(floatBytes, var.floatArray, 4);
}

void serialCommArdRasp::sendCharToArduino(char charToSend) {
	
	if (!communicationOpen) {
		#ifdef SER_DEBUG
			printf("Error: communication not opened.\nIf not done, you have to call init(char * usbDevice, unsigned int baudRate) before send any data.");
		#endif
		return;
	}

	if (!communicationInitialized) {
		#ifdef SER_DEBUG
			printf("Error: communication not initialized.\nDo it.");
		#endif
		return;
	}
	
	//serLock->lock();
	
	serialPutchar(communication, charToSend);
	

	//serLock->unlock();



}

void serialCommArdRasp::sendDataToArduino(float *numberToSend, int length) {
	
	if (!communicationOpen) {
		#ifdef SER_DEBUG
			printf("Error: communication not opened.\nIf not done, you have to call init(char * usbDevice, unsigned int baudRate) before send any data.");
		#endif 

		return;
	}

	if (!communicationInitialized) {
		#ifdef SER_DEBUG
			printf("Error: communication not initialized.\nDo it.");
		#endif 

		return;
	}
	
	//serLock->lock();
	
	for (int k = 0; k < length; k++){
		unsigned char floatBytes[4];
		float2Bytes(&floatBytes[0], numberToSend[k]);
	
		for (int i = 0; i<4; i++) {
			//fflush(stdout);
			serialPutchar(communication, floatBytes[i]);
		}
	}
	#ifdef SER_DEBUG_EXTREME
		printf("Mi metto in attesa di arduino\n"); 
		int cnt_ext = 1;
	#endif
	while (serialDataAvail(communication) <= 0) { /* wait for the arduino answer */ 
		#ifdef SER_DEBUG_EXTREME
			printf("[ %d ] Sto aspettando arduino\n", cnt_ext++); 
		#endif
	}
	
	char m = serialGetchar(communication); 
	while (m != 93) { // aspetto il carattere e svuoto il buffer,così mi risincronizzo
		while (serialDataAvail(communication) <= 0) { /* wait for the arduino answer */ }
		m = serialGetchar(communication); 
		#ifdef SER_DEBUG
			printf("Arrivato il carattere sbagliato: %c.\nAspetto il carattere e svuoto il buffer,così mi risincronizzo.\n", m);
		#endif
	}
	
	/*
	if(m != 93) { //ferma la comunicazione, non siamo sincronizzati
		printf("Arrivato il carattere sbagliato: %c.\n", m);
		communicationOpen = false;
		communicationInitialized = false;
	}
	*/
	//serLock->unlock();

}
