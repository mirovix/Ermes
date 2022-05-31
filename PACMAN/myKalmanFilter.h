#ifndef KALMAN
#pragma once
#include <mutex>
#include "serialCommArdRasp.hpp"

class MyKalmanFilter {
public:
	MyKalmanFilter();
	MyKalmanFilter(float **Q, float **R, float * state);

	void timeUpdate(float *imuMeasurement, float dt);
	void measurementUpdate(float *camMeasurement);
	void setQ(float **Q);
	void setR(float **R);
	void setState(float * state);
	float *getState();
	//float *getStateEstimated();
	void getAnglesRad(float * angles);
	void getAnglesDeg(float * angles);
	//void getAnglesEstRad(float * angles);
	//void getAnglesEstDeg(float * angles);
	//void printState();
	//void quaternionToAngles();

	serialCommArdRasp *serial;
private:

	void omegaFromMeasurements(float **omega, float *imuMeasurement);
	void scalarProductMatrix(float scalar, float **matrix);
	void matrixProductVector(float *vect, float **matrix);
	void matrixProductVector(float * result, float * vect, float ** matrix);
	void matrixProductMatrix(float **matrix, float **matrixA, float **matrixB);
	void matrixLessMatrix(float **matrixA, float **matrixB);
	void vectorLessVector(float *vectA, float *vectB);
	void vectorPlusVector(float * vect, float *vectA, float *vectB);
	void transposeMatrix(float **matrix, float ** matrixA);
	void sumMatrices(float **matrix, float **matrixA, float **matrixB);
	void inverseMatrix(float **matrix, float **mA);
	void assignIdentityMatrix(float **mat);
	void defineMatrices();

	float *state;
	float *angDeg;
	//float *stateEst;
	float **Q;
	float **R;
	float **P;
	//float **Pprojected;
	float **I;

	mutex filtMut;
};
#endif