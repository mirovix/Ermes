#include <stdio.h>
#include <stdlib.h>

#include <phidget22.h>

#ifndef _WIN32
#include <unistd.h>
#else
#include <Windows.h>
#endif


PhidgetSpatialHandle ch;
PhidgetReturnCode res;
const char *errs;

FILE* f;
char rslt_file_name[20] = "results.dat";


static void CCONV ssleep(int);

static void CCONV
onAttachHandler(PhidgetHandle phid, void *ctx) {
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

  if (hubPort == -1)
    printf("channel %d on device %d attached\n", channel, serial);
  else
    printf("channel %d on device %d hub port %d attached\n", channel, serial, hubPort);


  uint32_t minInterval, maxInterval;
  res = PhidgetSpatial_getMinDataInterval((PhidgetSpatialHandle)phid, &minInterval);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to get minInterval\n");
    return;
  }

  PhidgetSpatial_getMaxDataInterval((PhidgetSpatialHandle)phid, &maxInterval);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to get maxInterval\n");
    return;
  }
  
  printf("[ %d %d ]", minInterval, maxInterval);


  printf("\n\nI'm here!\n\n");

  res = PhidgetSpatial_setDataInterval((PhidgetSpatialHandle)phid, 10);
  // res = Phidget_setDataInterval(phid, 50);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed set dataInterval\n");
    hubPort = -1;
  }

  f = fopen(rslt_file_name,"w");
  
  if (f == NULL)
  {
    printf("Error opening file!\n");
    exit(1);
  }


  /*
  * Enable logging to stdout
  */
  PhidgetLog_enable(PHIDGET_LOG_INFO, NULL);


}

static void CCONV
onDetachHandler(PhidgetHandle phid, void *ctx) {
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

  if (hubPort != -1)
    printf("channel %d on device %d detached\n", channel, serial);
  else
    printf("channel %d on device %d hub port %d detached\n", channel, hubPort, serial);
}

static void CCONV
errorHandler(PhidgetHandle phid, void *ctx, Phidget_ErrorEventCode errorCode, const char *errorString) {

  fprintf(stderr, "Error: %s (%d)\n", errorString, errorCode);
}

static void CCONV
onSpatialData(PhidgetSpatialHandle ch, void *ctx, const double* acceleration, const double* angularRate, const double* magneticField, double timestamp) {

  printf("Timestamp: %.0f\n", timestamp);
  printf("acceleration Changed:%7.3f%8.3f%8.3f\n", acceleration[0], acceleration[1], acceleration[2]);
  printf("AngularRate Changed:%7.3f%8.3f%8.3f\n", angularRate[0], angularRate[1], angularRate[2]);
  printf("magneticField Changed:%7.3f%8.3f%8.3f\n", magneticField[0], magneticField[1], magneticField[2]);
  printf("\n");

  fprintf(f, "%.0f\t%6.4f\t%6.4f\t%6.4f\n", timestamp,
      acceleration[0], acceleration[1], acceleration[2]
  );
  
}


static void CCONV
Phidget_process(void){
  res = PhidgetSpatial_create(&ch);
  if (res != EPHIDGET_OK) {
    fprintf(stderr, "failed to create spatial channel\n");
    exit(1);
  }

  // res = initChannel((PhidgetHandle)ch);
  // if (res != EPHIDGET_OK) {
  //   Phidget_getErrorDescription(res, &errs);
  //   fprintf(stderr, "failed to initialize channel:%s\n", errs);
  //   exit(1);
  // }

  res = Phidget_setOnAttachHandler( (PhidgetHandle)ch, onAttachHandler, NULL);
   if (res != EPHIDGET_OK) {
    Phidget_getErrorDescription(res, &errs);
    fprintf(stderr, "failed to set onAttachHandler: %s\n", errs);
    goto done;
  }

  res = Phidget_setOnErrorHandler( (PhidgetHandle)ch, errorHandler, NULL);
   if (res != EPHIDGET_OK) {
    Phidget_getErrorDescription(res, &errs);
    fprintf(stderr, "failed to set onErrorHandler: %s\n", errs);
    goto done;
  }

  res = PhidgetSpatial_setOnSpatialDataHandler(ch, onSpatialData, NULL);
  if (res != EPHIDGET_OK) {
    Phidget_getErrorDescription(res, &errs);
    fprintf(stderr, "failed to set onSpatialDataHandler: %s\n", errs);
    goto done;
  }

  res = Phidget_setOnDetachHandler( (PhidgetHandle)ch, onDetachHandler, NULL);
  if (res != EPHIDGET_OK) {
    Phidget_getErrorDescription(res, &errs);
    fprintf(stderr, "failed to set onDetachHandler: %s\n", errs);
    goto done;
  }

  /*
  * Open the channel synchronously: waiting a maximum of 5 seconds.
  */
  res = Phidget_openWaitForAttachment((PhidgetHandle)ch, 5000);
  if (res != EPHIDGET_OK) {
    if (res == EPHIDGET_TIMEOUT) {
      printf("Channel did not attach after 5 seconds: please check that the device is attached\n");
    } else {
      Phidget_getErrorDescription(res, &errs);
      fprintf(stderr, "failed to open channel:%s\n", errs);
    }
    goto done;
  }

  printf("Gathering data for 10 seconds...\n");
  ssleep(10);

  done:

  Phidget_close((PhidgetHandle)ch);
  PhidgetSpatial_delete(&ch);

  fclose(f);

  exit(res);
}


static void CCONV
ssleep(int tm) {
#ifdef _WIN32
  Sleep(tm * 1000);
#else
  sleep(tm);
#endif
}


// todo link the method Process to an event handling it as a callback!