#include <stdlib.h>
#include <sys/time.h>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <unistd.h>
#include <fstream>
#include <stdio.h>
#include <thread>

#include <raspicam/raspicam_cv.h>

//#include "serialCommArdRasp.h"
#include "myKalmanFilter.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types.hpp"

#include "P3p.h"
#include <TooN/Cholesky.h>
#include <TooN/LU.h>

#include "global.h"

//#define DEBUG // to print information messages


#ifdef VISUALIZZAZIONE
	#define VISUALIZE
#else
	#undef VISUALIZE // to show images for debug purposes
#endif

#define DEBUG_PERFORMANCE
#define UNDISTORT_

#define FILENAME_SIZE 100

using namespace std;
using namespace cv;

class CAMERA 
{
	private:
		
		float frameWidth 	= 2.0*320.0;
		float frameHeight 	= 2.0*240.0;
		float expTime		= 0.08;
		
		float threshold_error = 15.0;
		
		unsigned int seconds = 1;
		int threshold_value = 140; //225; //158;
		int threshold_type = 0;
		int const max_BINARY_value = 255;
		
		double test_duration = 60.0;
		
		const unsigned int N_BLOBS = 5;
		
		bool isFirstFrame;
		
		const int PHI = 3;
		const int THETA = 4;
		const int PSI = 5;
		
		//raspicam::RaspiCam_Cv cam; 
		P3p p3p;
				
		Mat src, dst;
		
		TooN::Matrix<3,3> CM, invCM; // camera matrix and its inverse
		
		cv::Mat CMwrapped;
		cv::Mat dist_coeff;

		#ifdef VISUALIZE
			Mat rgb;
			// vector of colors to be used in plot
			std::vector<cv::Scalar> clr;	
		#endif
		
		vector<cv::Moments> mu;
		vector<cv::Point2f> mc;
		vector<cv::Point2f> mc_prev;
		
		
		// required matrices for p3p solver
		TooN::Matrix<5,3> worldPts;	
		TooN::Matrix<5,3> featureUnitVec;
		TooN::Matrix<3,16> p3pSol;
		TooN::Matrix<4> sol; // stores final pose solution
		
				
		std::vector<double> t;
		
		bool save_flag = false;
		char rslt_file_name[FILENAME_SIZE];
		FILE *f;
		
		struct timeval start;
		struct timeval curr_t;
		struct timeval time_elapsed;
		
		TooN::Vector<6> rel_state;
		TooN::Matrix<3,3> R_CI;
		TooN::Matrix<3,3> rel_att_in_imu_f;
		float rad = 135.0;
	
		
		//int test_number;
		
		std::vector< std::vector<cv::Point> > contours;
					
		void loadCameraMatrix( TooN::Matrix<3,3>& CM,
			TooN::Matrix<3,3>& invCM, double nPixX, double nPixY );
		void inv(TooN::Matrix<3>& m, TooN::Matrix<3>& minv);
		void getFeatUnitVec(TooN::Matrix<5,3>& u);
		void loadWorldPts( TooN::Matrix<5,3>& P );

		double getTimeElapsed(struct timeval t);

		void getFeatUnitVec(vector<cv::Point2f> mc, TooN::Matrix<3,3> invCM, 
								TooN::Matrix<5,3>& u);

		const string currentDateTime(void);
		void EnableDataSaving(void);
		void SaveData(void);

		void reproject3dto2d(TooN::Matrix<4> T, TooN::Vector<3> wPt, 
					TooN::Vector<3>& projPt);
		double norm(TooN::Vector<> v);
		void chooseP3pSol(TooN::Matrix<2,3> chk_w_pts, 
					TooN::Matrix<2,3> chk_f_pts, 
					double& rpj_err );
					
		void sortBlobs(void);
		void swap(unsigned int i, unsigned int j);
		double square_dist(Point2f p, Point2f q);
		double cross2d(Point2f p, Point2f q);

		void setRelState(void);
		
		bool isOutOfFrame(void);
		
		void setRelativeAttitudeInIMUFrame();
		
	public:
	
		CAMERA();
		
		//serialCommArdRasp *ser;
		MyKalmanFilter *filt;
		bool *camCommunicationON;
		void Run(void);
		
		//void Init(void);
		//void Process(void);

};
