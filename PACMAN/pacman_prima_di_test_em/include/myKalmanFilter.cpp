//#include "stdafx.h"
//#include <iostream>
//le librerie sopra sono da togliere
#include <math.h> 
#include "myKalmanFilter.h"

MyKalmanFilter::MyKalmanFilter() {
	defineMatrices();

	assignIdentityMatrix(I);
	assignIdentityMatrix(Q);
	assignIdentityMatrix(R);
	assignIdentityMatrix(P);
	//assignIdentityMatrix(Pprojected);

	state = new float[4];
	state[0] = 0;
	state[1] = 1;
	state[2] = 0;
	state[3] = 0;

	angDeg = new float[3];
	getAnglesDeg(angDeg);

	/*
	stateEst = new float[4];
	stateEst[0] = 1;
	stateEst[1] = 0;
	stateEst[2] = 0;
	stateEst[3] = 0;
	*/


};

MyKalmanFilter::MyKalmanFilter(float ** Q, float ** R, float * state) {
	MyKalmanFilter();
	setQ(Q);
	setR(R);
	setState(state);
};

void MyKalmanFilter::timeUpdate(float *imuMeasurement, float dt) {
	if(firstMeasureFromCamera) {
		if (filtMut.try_lock()) {

			float **omega = new float*[4];
			float **A = new float*[4];
			float **Atranspose = new float*[4];
			float **AP = new float*[4];
			float **APAt = new float*[4];

			for (int r = 0; r < 4; r++) {
				omega[r] = new float[4];
				A[r] = new float[4];
				Atranspose[r] = new float[4];
				AP[r] = new float[4];
				APAt[r] = new float[4];
			}
			/*------------Project state------------*/
			omegaFromMeasurements(omega, imuMeasurement);		// result in omega,			ricavo Omega
			scalarProductMatrix(0.5*dt, omega);					// result in omega,			calcolo 1/2*dt*Omega
			sumMatrices(A, I, omega);							// result in A,				calcolo A
			/*
			for (int r = 0; r < 4; r++) {
				for (int c = 0; c < 4; c++) {
					std::cout << A[r][c] << "\t";
				}
				std::cout << "\n";
			}
			*/
			matrixProductVector(state, A);						// result in state,			aggiorno lo stato
			/*----------Project Error Cov----------*/
			transposeMatrix(Atranspose, A);						// result in Atranspose,	calcolo prima A^T
			matrixProductMatrix(AP, A, P);						// result in AP,			calcolo AP
			matrixProductMatrix(APAt, AP, Atranspose);			// result in APAt,			calcolo APA^T
			sumMatrices(P, APAt, Q);							// result in P,				aggiorno P
			//sumMatrices(Pprojected, APAt, Q);	// result in Pprojected
			/*-------------------------------------*/

			getAnglesDeg(angDeg);
			
			serial->sendCharToArduino('[');
			serial->sendDataToArduino(angDeg, 2);

			filtMut.unlock();
		} else {
			printf("\t:(\n");
		}
	}
};

void MyKalmanFilter::measurementUpdate(float * camMeasurement) {
	filtMut.lock();
	
	if(!firstMeasureFromCamera) firstMeasureFromCamera = true;
 	
 	
 	
 	float **PR = new float*[4];
	float **PRinv= new float*[4];
	float **K = new float*[4];
	float **oldP = new float*[4];

	for (int r = 0; r < 4; r++) {
		PR[r] = new float[4];
		PRinv[r] = new float[4];
		K[r] = new float[4];
		oldP[r] = new float[4];
	}

	/*------------Kalman gain------------*/
	//sumMatrices(PR, Pprojected, R);// result in PR
	sumMatrices(PR, P, R);								// result in PR,				sommo le matrici P e R
	inverseMatrix(PRinv, PR);							// result in PRinv,				calcolo (P+R)^-1
	matrixProductMatrix(K, P, PRinv);					// result in K,					calcolo il guadagno K
	/*----------Update Estimate-----------*/
	vectorLessVector(camMeasurement, state);			// result in camMeasurement,	calcolo (z-x)
	matrixProductVector(camMeasurement, K);				// result in camMeasurement,	calcolo K(z-x)
	vectorPlusVector(state, state, camMeasurement);		// result in state,				aggiorno lo stato [x+K(z-x)]
	/*----------Update Error Cov----------*/
	matrixLessMatrix(I, K);								// result in I,					calcolo (I-K)
	
	for (int r = 0; r < 4; r++) {						//--------------------->		copio P in Pold ( P_{k-1} )
		for (int c = 0; c < 4; c++) {
			oldP[r][c] = P[r][c];
		}
	}
	matrixProductMatrix(P, I, oldP);					// result in P,					aggiorno P
	
	//matrixProductMatrix(P, I, Pprojected);// result in P
	/*-------------------------------------*/
	assignIdentityMatrix(I);							// ripristino I usata in precedenza per fare conti

	/*-------------------------------------*/
	getAnglesDeg(angDeg);
	serial->sendCharToArduino('[');
	serial->sendDataToArduino(angDeg, 2);

	filtMut.unlock();
};

void MyKalmanFilter::setQ(float ** Q) {
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			this->Q[r][c] = Q[r][c];
		}
	}
};

void MyKalmanFilter::setR(float ** R) {
	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			this->R[r][c] = R[r][c];
		}
	}
}
void MyKalmanFilter::setState(float * state) {
	for (int i = 0; i < 4; i++) {
		this->state[i] = state[i];
	}
	//matrixProductVector(stateEst, state, I);
};

float * MyKalmanFilter::getState() {
	return state;
}

/*
float * MyKalmanFilter::getStateEstimated() {
	return stateEst;
};
*/

void MyKalmanFilter::getAnglesRad(float * angles) {
	/*
	float rotMatrix[3][3];

	rotMatrix[0][0] = (2 * ((state[0] * state[0]) + (state[1] * state[1]))) - 1;
	rotMatrix[0][1] = 2 * ((state[1] * state[2]) - (state[0] * state[3]));
	rotMatrix[0][2] = 2 * ((state[1] * state[3]) + (state[0] * state[2]));
	rotMatrix[1][0] = 2 * ((state[1] * state[2]) + (state[0] * state[3]));
	rotMatrix[1][1] = (2 * ((state[0] * state[0]) + (state[2] * state[2]))) - 1;
	rotMatrix[1][2] = 2 * ((state[2] * state[3]) - (state[0] * state[1]));
	rotMatrix[2][0] = 2 * ((state[1] * state[3]) - (state[0] * state[2]));
	rotMatrix[2][1] = 2 * ((state[2] * state[3]) + (state[0] * state[1]));
	rotMatrix[2][2] = (2 * ((state[0] * state[0]) + (state[3] * state[3]))) - 1;

	angles[0] = atan2(rotMatrix[1][0], rotMatrix[0][0]);
	angles[1] = atan2(-rotMatrix[2][0], sqrt((rotMatrix[2][1] * rotMatrix[2][1]) + (rotMatrix[2][2] * rotMatrix[2][2])));
	angles[2] = atan2(rotMatrix[2][1], rotMatrix[2][2]);

	*/
	angles[0] = atan2(2 * ((state[0] * state[1]) + (state[2] * state[3])), 1 - (2 * ((state[1] * state[1]) + (state[2] * state[2]))));
	angles[1] = asin(2 * ((state[0] * state[2]) - (state[3] * state[1])));
	angles[2] = atan2(2 * ((state[0] * state[3]) + (state[1] * state[2])), 1 - (2 * ((state[2] * state[2]) + (state[3] * state[3]))));
	
};

void MyKalmanFilter::getAnglesDeg(float * angles) {

	getAnglesRad(angles);

	for (int i = 0; i < 3; i++) {
		angles[i] = angles[i] * 180 / 3.14;
	}

};

/*
void MyKalmanFilter::getAnglesEstRad(float * angles) {
	
	//float rotMatrix[3][3];

	//rotMatrix[0][0] = (2 * ((stateEst[0] * stateEst[0]) + (stateEst[1] * stateEst[1]))) - 1;
	//rotMatrix[0][1] = 2 * ((stateEst[1] * stateEst[2]) - (stateEst[0] * stateEst[3]));
	//rotMatrix[0][2] = 2 * ((stateEst[1] * stateEst[3]) + (stateEst[0] * stateEst[2]));
	//rotMatrix[1][0] = 2 * ((stateEst[1] * stateEst[2]) + (stateEst[0] * stateEst[3]));
	//rotMatrix[1][1] = (2 * ((stateEst[0] * stateEst[0]) + (stateEst[2] * stateEst[2]))) - 1;
	//rotMatrix[1][2] = 2 * ((stateEst[2] * stateEst[3]) - (stateEst[0] * stateEst[1]));
	//rotMatrix[2][0] = 2 * ((stateEst[1] * stateEst[3]) - (stateEst[0] * stateEst[2]));
	//rotMatrix[2][1] = 2 * ((stateEst[2] * stateEst[3]) + (stateEst[0] * stateEst[1]));
	//rotMatrix[2][2] = (2 * ((stateEst[0] * stateEst[0]) + (stateEst[3] * stateEst[3]))) - 1;

	//angles[0] = atan2(rotMatrix[1][0], rotMatrix[0][0]);
	//angles[1] = atan2(-rotMatrix[2][0], sqrt((rotMatrix[2][1] * rotMatrix[2][1]) + (rotMatrix[2][2] * rotMatrix[2][2])));
	//angles[2] = atan2(rotMatrix[2][1], rotMatrix[2][2]);
	
	angles[0] = atan2(2 * ((stateEst[0] * stateEst[1]) + (stateEst[2] * stateEst[3])), 1 - (2 * ((stateEst[1] * stateEst[1]) + (stateEst[2] * stateEst[2]))));
	angles[1] = asin(2 * ((stateEst[0] * stateEst[2]) - (stateEst[3] * stateEst[1])));
	angles[2] = atan2(2 * ((stateEst[0] * stateEst[3]) + (stateEst[1] * stateEst[2])), 1 - (2 * ((stateEst[2] * stateEst[2]) + (stateEst[3] * stateEst[3]))));
	
};

void MyKalmanFilter::getAnglesEstDeg(float * angles) {

	getAnglesEstRad(angles);

	for (int i = 0; i < 3; i++) {
		angles[i] = angles[i] * 180 / 3.14;
	}

};


void MyKalmanFilter::printState() {
	for (int i = 0; i < 4; i++) {
		std::cout << state[i] << "\t";
	}
	std::cout << "\n";
};
*/

void MyKalmanFilter::omegaFromMeasurements(float **omega, float *imuMeasurement) { // result in omega
	
	//float omega[4][4];
	omega[0][0] = 0;
	omega[0][1] = -imuMeasurement[0];	//-w_x
	omega[0][2] = -imuMeasurement[1];	//-w_y
	omega[0][3] = -imuMeasurement[2];	//-w_z
	omega[1][0] = imuMeasurement[0];	//w_x
	omega[1][1] = 0;
	omega[1][2] = imuMeasurement[2];	//w_z
	omega[1][3] = -imuMeasurement[1];	//-w_y
	omega[2][0] = imuMeasurement[1];	//w_y
	omega[2][1] = -imuMeasurement[2];	//-w_z
	omega[2][2] = 0;
	omega[2][3] = imuMeasurement[0];	//w_x
	omega[3][0] = imuMeasurement[2];	//w_z
	omega[3][1] = imuMeasurement[1];	//w_y
	omega[3][2] = -imuMeasurement[0];	//-w_x
	omega[3][3] = 0;

}
/*
void MyKalmanFilter::quaternionToAngles() {
	float rotMatrix[3][3];
	
	rotMatrix[0][0] = (2 * ((state[0] * state[0]) + (state[1] * state[1])))-1;
	rotMatrix[0][1] = 2 * ((state[1] * state[2]) - (state[0] * state[3]));
	rotMatrix[0][2] = 2 * ((state[1] * state[3]) + (state[0] * state[2]));
	rotMatrix[1][0] = 2 * ((state[1] * state[2]) + (state[0] * state[3]));
	rotMatrix[1][1] = (2 * ((state[0] * state[0]) + (state[2] * state[2]))) - 1;
	rotMatrix[1][2] = 2 * ((state[2] * state[3]) - (state[0] * state[1]));
	rotMatrix[2][0] = 2 * ((state[1] * state[3]) - (state[0] * state[2]));
	rotMatrix[2][1] = 2 * ((state[2] * state[3]) + (state[0] * state[1]));
	rotMatrix[2][2] = (2 * ((state[0] * state[0]) + (state[3] * state[3]))) - 1;

	float * eulerAngles = new float[3];
	
	eulerAngles[0] = atan2(rotMatrix[1][0], rotMatrix[0][0]);
	eulerAngles[1] = atan2(-rotMatrix[2][0], sqrt((rotMatrix[2][1] * rotMatrix[2][1])+(rotMatrix[2][2] * rotMatrix[2][2])));
	eulerAngles[2] = atan2(rotMatrix[2][1], rotMatrix[2][2]);

	std::cout << eulerAngles[0] << "\t" << eulerAngles[1] << "\t" << eulerAngles[2] << "\n";

};
*/
void MyKalmanFilter::scalarProductMatrix(float scalar, float **matrix) { // result in matrix

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrix[r][c] = matrix[r][c] * scalar;
		}
	}

};

void MyKalmanFilter::matrixProductVector(float * vect, float ** matrix) { // result in vect
	float newState[4];
	for (int i = 0;i < 4; i++) {
		newState[i] = 0;
	}

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			newState[r] += vect[c] * matrix[r][c];
		}
	}

	for (int i = 0; i < 4; i++) {
		vect[i] = newState[i];
	}
};

void MyKalmanFilter::matrixProductVector(float * result, float * vect, float ** matrix) { // result in result
	float newState[4];
	for (int i = 0; i < 4; i++) {
		newState[i] = 0;
	}

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			newState[r] += vect[c] * matrix[r][c];
		}
	}

	for (int i = 0; i < 4; i++) {
		result[i] = newState[i];
	}
};

void MyKalmanFilter::matrixProductMatrix(float ** matrix, float ** matrixA, float ** matrixB) { // result in matrix

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrix[r][c] = 0;
		}
	}

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrix[r][c] += matrixA[r][c] * matrixB[c][r];
		}
	}

}
void MyKalmanFilter::matrixLessMatrix(float ** matrixA, float ** matrixB) { //result in matrixA

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrixA[r][c] -= matrixB[r][c];
		}
	}

};

void MyKalmanFilter::vectorLessVector(float * vectA, float * vectB) { //result in vectA

	for (int i = 0; i < 4; i++) {
		vectA[i] -= vectB[i];
	}

};

void MyKalmanFilter::vectorPlusVector(float * vect, float * vectA, float * vectB) { //result in vect

	for (int i = 0; i < 4; i++) {
		vect[i] = vectA[i] + vectB[i];
	}

};

void MyKalmanFilter::transposeMatrix(float ** matrix, float ** matrixA) {

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrix[r][c] = matrixA[c][r];
		}
	}


};

void MyKalmanFilter::sumMatrices(float **matrix, float ** matrixA, float ** matrixB) {

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrix[r][c] = matrixA[r][c] + matrixB[r][c];
		}
	}

}
void MyKalmanFilter::inverseMatrix(float ** matrix, float ** mA) {

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			matrix[r][c] = 0;
		}
	}

	float determinant = (mA[0][0] * mA[1][1] * mA[2][2] * mA[3][3]) + (mA[0][0] * mA[1][2] * mA[2][3] * mA[3][1]) + (mA[0][0] * mA[1][3] * mA[2][1] * mA[3][2]) +
						(mA[0][1] * mA[1][0] * mA[2][3] * mA[3][2]) + (mA[0][1] * mA[1][2] * mA[2][0] * mA[3][3]) + (mA[0][1] * mA[1][3] * mA[2][2] * mA[3][0]) +
						(mA[0][2] * mA[1][0] * mA[2][1] * mA[3][3]) + (mA[0][2] * mA[1][1] * mA[2][3] * mA[3][0]) + (mA[0][2] * mA[1][3] * mA[2][0] * mA[3][1]) +
						(mA[0][3] * mA[1][0] * mA[2][2] * mA[3][1]) + (mA[0][3] * mA[1][1] * mA[2][0] * mA[3][2]) + (mA[0][3] * mA[1][2] * mA[2][1] * mA[3][0]) -
						(mA[0][0] * mA[1][1] * mA[2][3] * mA[3][2]) - (mA[0][0] * mA[1][2] * mA[2][1] * mA[3][3]) - (mA[0][0] * mA[1][3] * mA[2][2] * mA[3][1]) -
						(mA[0][1] * mA[1][0] * mA[2][2] * mA[3][3]) - (mA[0][1] * mA[1][2] * mA[2][3] * mA[3][0]) - (mA[0][1] * mA[1][3] * mA[2][0] * mA[3][2]) -
						(mA[0][2] * mA[1][0] * mA[2][3] * mA[3][1]) - (mA[0][2] * mA[1][1] * mA[2][0] * mA[3][3]) - (mA[0][2] * mA[1][3] * mA[2][1] * mA[3][0]) -
						(mA[0][3] * mA[1][0] * mA[2][1] * mA[3][2]) - (mA[0][3] * mA[1][1] * mA[2][2] * mA[3][0]) - (mA[0][3] * mA[1][2] * mA[2][0] * mA[3][1]);

	if (determinant == 0) return; //matrice non invertibile

	matrix[0][0] = (mA[1][1] * mA[2][2] * mA[3][3]) + (mA[1][2] * mA[2][3] * mA[3][1]) + (mA[1][3] * mA[2][1] * mA[3][2]) - (mA[1][1] * mA[2][3] * mA[3][2]) - (mA[1][2] * mA[2][1] * mA[3][3]) - (mA[1][3] * mA[2][2] * mA[3][1]);
	matrix[0][1] = (mA[0][1] * mA[2][3] * mA[3][2]) + (mA[0][2] * mA[2][1] * mA[3][3]) + (mA[0][3] * mA[2][2] * mA[3][1]) - (mA[0][1] * mA[2][2] * mA[3][3]) - (mA[0][2] * mA[2][3] * mA[3][1]) - (mA[0][3] * mA[2][1] * mA[3][2]);
	matrix[0][2] = (mA[0][1] * mA[1][2] * mA[3][3]) + (mA[0][2] * mA[1][3] * mA[3][1]) + (mA[0][3] * mA[1][1] * mA[3][2]) - (mA[0][1] * mA[1][3] * mA[3][2]) - (mA[0][2] * mA[1][1] * mA[3][3]) - (mA[0][3] * mA[1][2] * mA[3][1]);
	matrix[0][3] = (mA[0][1] * mA[1][3] * mA[2][2]) + (mA[0][2] * mA[1][1] * mA[2][3]) + (mA[0][3] * mA[1][2] * mA[2][1]) - (mA[0][1] * mA[1][2] * mA[2][3]) - (mA[0][2] * mA[1][3] * mA[2][1]) - (mA[0][3] * mA[1][1] * mA[2][2]);
	matrix[1][0] = (mA[1][0] * mA[2][3] * mA[3][2]) + (mA[1][2] * mA[2][0] * mA[3][3]) + (mA[1][3] * mA[2][2] * mA[3][0]) - (mA[1][0] * mA[2][2] * mA[3][3]) - (mA[1][2] * mA[2][3] * mA[3][0]) - (mA[1][3] * mA[2][0] * mA[3][2]);
	matrix[1][1] = (mA[0][0] * mA[2][2] * mA[3][3]) + (mA[0][2] * mA[2][3] * mA[3][0]) + (mA[0][3] * mA[2][0] * mA[3][2]) - (mA[0][0] * mA[2][3] * mA[3][2]) - (mA[0][2] * mA[2][0] * mA[3][3]) - (mA[0][3] * mA[2][2] * mA[3][0]);
	matrix[1][2] = (mA[0][0] * mA[1][3] * mA[3][2]) + (mA[0][2] * mA[1][0] * mA[3][3]) + (mA[0][3] * mA[1][2] * mA[3][0]) - (mA[0][0] * mA[1][2] * mA[3][3]) - (mA[0][2] * mA[1][3] * mA[3][0]) - (mA[0][3] * mA[1][0] * mA[3][2]);
	matrix[1][3] = (mA[0][0] * mA[1][2] * mA[2][3]) + (mA[0][2] * mA[1][3] * mA[2][0]) + (mA[0][3] * mA[1][0] * mA[2][2]) - (mA[0][0] * mA[1][3] * mA[2][2]) - (mA[0][2] * mA[1][0] * mA[2][3]) - (mA[0][3] * mA[1][2] * mA[2][0]);
	matrix[2][0] = (mA[1][0] * mA[2][1] * mA[3][3]) + (mA[1][1] * mA[2][3] * mA[3][0]) + (mA[1][3] * mA[2][0] * mA[3][1]) - (mA[1][0] * mA[2][3] * mA[3][1]) - (mA[1][1] * mA[2][0] * mA[3][3]) - (mA[1][3] * mA[2][1] * mA[3][0]);
	matrix[2][1] = (mA[0][0] * mA[2][3] * mA[3][1]) + (mA[0][1] * mA[2][0] * mA[3][3]) + (mA[0][3] * mA[2][1] * mA[3][0]) - (mA[0][0] * mA[2][1] * mA[3][3]) - (mA[0][1] * mA[2][3] * mA[3][0]) - (mA[0][3] * mA[2][0] * mA[3][1]);
	matrix[2][2] = (mA[0][0] * mA[1][1] * mA[3][3]) + (mA[0][1] * mA[1][3] * mA[3][0]) + (mA[0][3] * mA[1][0] * mA[3][1]) - (mA[0][0] * mA[1][3] * mA[3][1]) - (mA[0][1] * mA[1][0] * mA[3][3]) - (mA[0][3] * mA[1][1] * mA[3][0]);
	matrix[2][3] = (mA[0][0] * mA[1][3] * mA[2][1]) + (mA[0][1] * mA[1][0] * mA[2][3]) + (mA[0][3] * mA[1][1] * mA[2][0]) - (mA[0][0] * mA[1][1] * mA[2][3]) - (mA[0][1] * mA[1][3] * mA[2][0]) - (mA[0][3] * mA[1][0] * mA[2][1]);
	matrix[3][0] = (mA[1][0] * mA[2][2] * mA[3][1]) + (mA[1][1] * mA[2][0] * mA[3][2]) + (mA[1][2] * mA[2][1] * mA[3][0]) - (mA[1][0] * mA[2][1] * mA[3][2]) - (mA[1][1] * mA[2][2] * mA[3][0]) - (mA[1][2] * mA[2][0] * mA[3][1]);
	matrix[3][1] = (mA[0][0] * mA[2][1] * mA[3][2]) + (mA[0][1] * mA[2][2] * mA[3][0]) + (mA[0][2] * mA[2][0] * mA[3][1]) - (mA[0][0] * mA[2][2] * mA[3][1]) - (mA[0][1] * mA[2][0] * mA[3][2]) - (mA[0][2] * mA[2][1] * mA[3][0]);
	matrix[3][2] = (mA[0][0] * mA[1][2] * mA[3][1]) + (mA[0][1] * mA[1][0] * mA[3][2]) + (mA[0][2] * mA[1][1] * mA[3][0]) - (mA[0][0] * mA[1][1] * mA[3][2]) - (mA[0][1] * mA[1][2] * mA[3][0]) - (mA[0][2] * mA[1][0] * mA[3][1]);
	matrix[3][3] = (mA[0][0] * mA[1][1] * mA[2][2]) + (mA[0][1] * mA[1][2] * mA[2][0]) + (mA[0][2] * mA[1][0] * mA[2][1]) - (mA[0][0] * mA[1][2] * mA[2][1]) - (mA[0][1] * mA[1][0] * mA[2][2]) - (mA[0][2] * mA[1][1] * mA[2][0]);

	scalarProductMatrix(1/determinant, matrix);

};

void MyKalmanFilter::assignIdentityMatrix(float **mat) {

	for (int r = 0; r < 4; r++) {
		for (int c = 0; c < 4; c++) {
			if(r==c) mat[r][c] = 1;
			else mat[r][c] = 0;
		}
	}
};

void MyKalmanFilter::defineMatrices() {
	Q = new float*[4];
	R = new float*[4];
	P = new float*[4];
	//Pprojected = new float*[4];
	I = new float*[4];

	for (int r = 0; r < 4; r++) {
		Q[r] = new float[4];
		R[r] = new float[4];
		P[r] = new float[4];
		//Pprojected[r] = new float[4];
		I[r] = new float[4];
	}

};
