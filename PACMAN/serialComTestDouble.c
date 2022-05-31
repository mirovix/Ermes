#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringSerial.h>

void float2Bytes(unsigned char* floatBytes, float number) {
	union {
		float floatVar;
		unsigned char floatArray[4];
	} var;
	var.floatVar = number;
	//memcpy(floatBytes, (unsigned char*)(&number), 4);
	memcpy(floatBytes, var.floatArray, 4);
}


int main()
{
	int fd;
	float nToSend = 1.56489;
	unsigned char floatBytes[4];
	int i = 0;

	if ((fd = serialOpen("/dev/serial0", 115200)) < 0)
	{
		fprintf(stderr, "Unable to open serial device: %s\n", strerror(errno));
		return 1;
	}

	unsigned long sendTime = micros();
	unsigned long receiveTime;

	printf("Initializing the transmission, sending the starting character...\n");
	fflush(stdout);
	serialPutchar(fd, 's');
    
	while (serialDataAvail(fd) <= 0) { /*wait for the arduino answer*/ }

	receiveTime = micros();
	printf("total waited time: %6d us\n", receiveTime-sendTime);	
	delay(1000);
	if (serialGetchar(fd) == 107) {

		for(;;){
			sendTime = micros();
			printf("Sending %9d at %9d us, receiving: ", nToSend, sendTime);
			float2Bytes(&floatBytes[0], nToSend);
			for(i = 0; i<4;i++) {
				fflush(stdout);
				serialPutchar(fd, floatBytes[i]);
			}
			
			while (serialDataAvail(fd) <= 0) { /*wait for the arduino answer*/ }
			receiveTime = micros();
			printf("%1c at %9d us, for a total of %6d us\n", serialGetchar(fd), receiveTime, receiveTime-sendTime); 
			fflush(stdout);
		}

	}
	
	return 0;
}

