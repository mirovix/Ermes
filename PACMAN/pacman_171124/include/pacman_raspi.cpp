#include "pacman_raspi.hpp"

// ---------------------------------------------------------------------
CAMERA::CAMERA(){
	
	
	R_CI[0][0] = cos(rad);
	R_CI[0][1] = -sin(rad);
	R_CI[0][2] = 0.0;
	R_CI[1][0] = sin(rad);
	R_CI[1][1] = cos(rad);
	R_CI[1][2] = 0.0;
	R_CI[2][0] = 0.0;
	R_CI[2][1] = 0.0;
	R_CI[2][2] = 1.0;
}

// ---------------------------------------------------------------------
void CAMERA::loadCameraMatrix( TooN::Matrix<3,3>& CM, TooN::Matrix<3,3>& invCM, 
		double nPixX, double nPixY )
{
	#ifdef DEBUG
		cout << "Loading camera matrix ...";
	#endif
	
	CM[0] = TooN::makeVector(508.9298, 0.0, 323.9868);
	CM[1] = TooN::makeVector(0.0, 508.8157, 249.2341);
	CM[2] = TooN::makeVector(0.0, 0.0, 1.0);
	
	// compute scale factor
	int sf_x = (int)nPixX/320;
	int sf_y = (int)nPixY/240;
	
	if ( sf_x != sf_y ){ cerr << "Something went wrong in loadCameraMatrix" << endl; }
	else
	{
		switch(sf_x)
		{
			case 1:
				CM[0] = CM[0]/2;
				CM[1] = CM[1]/2;
				break;
			case 2:
				break;
			case 4:
				CM[0] = CM[0]*2;
				CM[1] = CM[1]*2;
				break;
			default:
				cout << "sf_x = " << sf_x << endl;
				cerr << "Too big I think! Decrease resolution" << endl;
				break;
		}
	}

	// compute inv(CM)
	inv(CM, invCM);
		
	// init dist_coeffs
	// assunzione skew=0
	// info: https://docs.opencv.org/3.3.0/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e
	dist_coeff.push_back(  0.194837719473325); //k1
	dist_coeff.push_back( -0.472192045424245); //k2
	dist_coeff.push_back( -0.001895461231085); //p1
	dist_coeff.push_back( -0.001363026805053); //p2
	dist_coeff.push_back(  0.253892214018371); //k3

	


	CMwrapped = Mat(3,3,CV_64FC1);
	for (int i=0; i < 3; i++){
		for(int j=0; j<3; j++)
			CMwrapped.at<double>(i,j) = CM[i][j];
		
	}
	
	cout << "SUCCESS" << endl;
}


// ---------------------------------------------------------------------
void CAMERA::inv(TooN::Matrix<3>& m, TooN::Matrix<3>& minv)
{
	// computes the inverse of a matrix m
	double det = m(0, 0) * (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) -
	             m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
	             m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
	
	double invdet = 1 / det;
	
	minv(0, 0) = (m(1, 1) * m(2, 2) - m(2, 1) * m(1, 2)) * invdet;
	minv(0, 1) = (m(0, 2) * m(2, 1) - m(0, 1) * m(2, 2)) * invdet;
	minv(0, 2) = (m(0, 1) * m(1, 2) - m(0, 2) * m(1, 1)) * invdet;
	minv(1, 0) = (m(1, 2) * m(2, 0) - m(1, 0) * m(2, 2)) * invdet;
	minv(1, 1) = (m(0, 0) * m(2, 2) - m(0, 2) * m(2, 0)) * invdet;
	minv(1, 2) = (m(1, 0) * m(0, 2) - m(0, 0) * m(1, 2)) * invdet;
	minv(2, 0) = (m(1, 0) * m(2, 1) - m(2, 0) * m(1, 1)) * invdet;
	minv(2, 1) = (m(2, 0) * m(0, 1) - m(0, 0) * m(2, 1)) * invdet;
	minv(2, 2) = (m(0, 0) * m(1, 1) - m(1, 0) * m(0, 1)) * invdet;
}

//----------------------------------------------------------------------
void CAMERA::loadWorldPts( TooN::Matrix<5,3>& P )
{
	double p0x, p0y, p0z,
			p1x, p1y, p1z,
			p2x, p2y, p2z,
			p3x, p3y, p3z,
			p4x, p4y, p4z;
	

	/*
	// first pt
	p0x = -8.0;
	p0y = -29.0;
	p0z = 0.0;
	// second pt
	p1x = -8.0;
	p1y = 30.0;
	p1z = 0.0;
	// third pt
	p2x = 30.0;
	p2y = -12.0;
	p2z = 0.0;
	// fourth pt
	p3x = -30.0;
	p3y = -12.0;
	p3z = 0.0;
	//fifth point
	p4x = -8.0;
	p4y = -4.0;
	p4z = -10.0;
	*/
	
	// first pt
	p0x = -8.0;
	p0y = 29.0;
	p0z = 0.0;
	// second pt
	p1x = -8.0;
	p1y = -30.0;
	p1z = 0.0;
	// third pt
	p2x = 30.0;
	p2y = 12.0;
	p2z = 0.0;
	// fourth pt
	p3x = -30.0;
	p3y = 12.0;
	p3z = 0.0;
	//fifth point
	p4x = -8.0;
	p4y = 4.0;
	p4z = 10.0;
	
	P[0] = TooN::makeVector(p0x, p0y, p0z);
	P[1] = TooN::makeVector(p1x, p1y, p1z);
	P[2] = TooN::makeVector(p2x, p2y, p2z);
	P[3] = TooN::makeVector(p3x, p3y, p3z);
	P[4] = TooN::makeVector(p4x, p4y, p4z);
}


//----------------------------------------------------------------------
double CAMERA::getTimeElapsed(struct timeval t)
{
	return t.tv_sec + 1e-6 * t.tv_usec;
}


// ---------------------------------------------------------------------
void CAMERA::getFeatUnitVec(TooN::Matrix<5,3>& u)
{
	TooN::Vector<3,double> featV;
	
	for (int i=0; i<5; i++)
	{	
		featV = TooN::makeVector( (double)mc[i].x, (double)mc[i].y, 1);
		u[i] = unit(invCM * featV); // get vector and normalize
	}
}


// ---------------------------------------------------------------------
void CAMERA::chooseP3pSol(TooN::Matrix<2,3> chk_w_pts, 
					TooN::Matrix<2,3> chk_f_pts, 
					double& rpj_err )
{
	double reproj_err = 1000.0;
	double mean_err = reproj_err + 1.0;
	int index=-1;
	
	TooN::Matrix<4> T = TooN::Identity;
	//cout << "T = " << T << endl << endl;
	
	for ( int count = 0; count < 4; count++ )
	{
		TooN::Matrix<2,3> projectedPt;
				
		T.slice<0,0,3,3>() = p3pSol.slice(0, count*4+1, 3, 3).T();
		T.slice<0,3,3,1>() = - T.slice<0,0,3,3>() * p3pSol.slice(0, count*4, 3, 1);
		
		for (int i=0; i<2; i++){
			TooN::Vector<3> tmp;
			reproject3dto2d(T, 
				TooN::makeVector(chk_w_pts[i][0], chk_w_pts[i][1], chk_w_pts[i][2] ), 
				tmp
				);
			projectedPt[i] = tmp;
		}
		
		double err = 0.0;
		for (int i=0; i<2; i++){
			err = err + norm( projectedPt[i] - chk_f_pts[i] );
			
			//out << count<< "\t" << getTimeElapsed(time_elapsed) << "\terr " << i << " = " << norm( projectedPt[i] - chk_f_pts[i] ) << endl;
			
		}
		mean_err = err/2;
		
		if ( mean_err < reproj_err )
		{
			reproj_err = mean_err;
			index = count;
		}
	}
	//cout << "reproj_err = " << reproj_err << endl;
	#ifdef DEBUG
		cout << "reproj_err = " << reproj_err << endl;
	#endif
		
	if (reproj_err > threshold_error){
		//cout << "Reprojection error is too high ----------------------->  " << reproj_err << endl << endl;
	} else {
		sol.slice<0,0,3,3>() = p3pSol.slice(0,index*4+1,3,3).T();
		sol.slice<0,3,3,1>() = - sol.slice(0,0,3,3) * p3pSol.slice(0,index*4,3,1);
		sol[3] = T[3];
		//rpj_err = reproj_err;
	}
	
	rpj_err = reproj_err;
}

// ---------------------------------------------------------------------
double CAMERA::norm(TooN::Vector<> v)
{
	double n = 0.0;
	
	//cout << "v.size = " << v.size() << endl;
	for (int i=0; i<v.size(); i++)
		n += v[i]*v[i];
		
	//cout << "n = " << n << endl;
	return sqrt(n);
}


// ---------------------------------------------------------------------

void CAMERA::setRelativeAttitudeInIMUFrame() {
	
	rel_att_in_imu_f = R_CI * sol.slice<0,0,3,3>();
	
	rel_state[3] = atan2(rel_att_in_imu_f(1,2),rel_att_in_imu_f(2,2));
	rel_state[4] =	-asin(rel_att_in_imu_f(0,2));
	rel_state[5] =	atan2(rel_att_in_imu_f(0,1),rel_att_in_imu_f(0,0));
	
	cout << "rel_state_imu_frame: " << rel_state << endl;
	
}

void CAMERA::setRelState(void)
{
	rel_state = TooN::makeVector(
		sol(0,3),
		sol(1,3),
		sol(2,3),
		atan2(sol(1,2),sol(2,2)),
		-asin(sol(0,2)),
		atan2(sol(0,1),sol(0,0))
	);
	printf("non girata:\t[%f, %f]", rel_state[3], rel_state[4]);
}


// ---------------------------------------------------------------------
void CAMERA::reproject3dto2d(TooN::Matrix<4> T, TooN::Vector<3> wPt, 
					TooN::Vector<3>& projPt)
{
	TooN::Vector<4> v = TooN::makeVector(wPt[0], wPt[1], wPt[2], 1);
	TooN::Vector<4> v1 = T*v; // wPt in camera frame
		
	TooN::Vector<3> v2 = TooN::makeVector(v1[0]/v1[2], v1[1]/v1[2], 1);
	
	projPt = CM * v2;
}

// ---------------------------------------------------------------------
void CAMERA::sortBlobs(void)
{
	vector<cv::Point2f> tmp(5);

	float min_x = 1000.0;
	float max_x = 0.0;
	float min_y = 1000.0;
	float max_y = 0.0;

	int min_x_ind, min_y_ind, max_x_ind, max_y_ind, mid_ind;

	if(isFirstFrame)
	{
		
		for(unsigned int i=0; i<N_BLOBS; i++)
		{
			if( mc[i].x < min_x ){ min_x = mc[i].x; min_x_ind = i; }
			if( mc[i].y < min_y ){ min_y = mc[i].y; min_y_ind = i; }
			if( mc[i].x > max_x ){ max_x = mc[i].x; max_x_ind = i; }
			if( mc[i].y > max_y ){ max_y = mc[i].y; max_y_ind = i; }
		}
		
		for (unsigned int i=0; i<N_BLOBS; i++)
			if( mc[i].x > min_x && mc[i].x < max_x && mc[i].y < max_y && mc[i].y > min_y ) {mid_ind = i;}
		
		tmp[0] = mc[max_y_ind];
		tmp[1] = mc[min_y_ind];
		tmp[2] = mc[max_x_ind];
		tmp[3] = mc[min_x_ind];
		tmp[4] = mc[mid_ind];	
		
		for(unsigned int i=0; i<N_BLOBS; i++)	mc[i] = tmp[i];
		
		isFirstFrame = false;
	}
	else
	{
		
		TooN::Matrix<5,5,float> d;
		// calc distance of mc[i] from mc_prev[j]
		for( int i=0; i<5; i++ )
		{
			for( int j=0; j<5; j++ )
			{
				d[i][j] = square_dist(mc[i],mc_prev[j]);
			}
		}
		
		// find indexes of min distances
		vector<int> minindex(5);
		for( int i=0; i<5; i++ )
		{
			float mindist = 100.0;
			for( int j=0; j<5; j++ )
			{
				if (d[i][j] < mindist) {mindist = d[i][j]; minindex[i] = j;} 
			}
		}
		/*
		cout << "minindex = ";
		for( int i=0; i<5; i++ )
			cout << minindex[i] << "\t";
			
		cout << endl;
		*/
		// and finallly sort elements
		
		vector<Point2f> buf(5);
		for( int i=0; i<5; i++)
			buf[ minindex[i] ] = mc[ i ];
		
		
		for( int i=0; i<5; i++)
			mc[i] = buf[i];
		
		
		/*
		//waitKey(10000);
		
		
		//for(unsigned int i=0; i<N_BLOBS; i++)
		//{
			//if( mc[i].x < min_x ){ min_x = mc[i].x; }
			//if( mc[i].y < min_y ){ min_y = mc[i].y; }
			//if( mc[i].x > max_x ){ max_x = mc[i].x; }
			//if( mc[i].y > max_y ){ max_y = mc[i].y; }
		//}
		
		//for(unsigned int i=0; i<N_BLOBS; i++)
		//{
			//if( mc[i].x > min_x && mc[i].x < max_x	&& mc[i].y > min_y && mc[i].y < max_y) {
				//mid_ind = i;
			//}
		//}		
		//swap(mid_ind,4); // assign pt [i] to index 4 (central pt of the target)
		
		//// determine which is the pt of index 0 in the new frame
		//double tmp;
		//double min_dist = 100.0;
		//int ind;
		//for(unsigned int i=0; i<N_BLOBS; i++){
		
			//tmp = square_dist(mc[i], mc_prev[0]);
			//if( tmp < min_dist ) {
				//min_dist = tmp;
				//ind = i;
			//}
		//}
		//swap(ind,0);
	
		
		
		
		
		
		//// compute twice the area of triangles formed by leds
		//bool a = (cross2d( (mc[1]-mc[0]) , (mc[2]-mc[0]) ) > 0);
		//bool b = (cross2d( (mc[1]-mc[0]) , (mc[3]-mc[0]) ) > 0);
		//bool c = (cross2d( (mc[2]-mc[1]) , (mc[3]-mc[1]) ) > 0);
		
		//cout << "a = " << a ;
		//cout << "\tb = " << b ;
		//cout << "\tc = " << c << endl << endl;
		
		////waitKey(1e3);
		
		//if( a  &  b &  c )	{swap(1,2); swap(2,3); goto done;}
		//if( a  &  b & ~c )	{swap(1,3); goto done;}
		//if( ~a &  b & ~c )	{swap(2,3); goto done;}
		//if( ~a & ~b &  c )	{swap(2,3); swap(1,2); goto done;}
		//if(  a & ~b &  c )	{goto done;}
		//if( ~a & ~b & ~c )	{swap(1,2); goto done;}
		
		//done: return;
		*/
	}
	
}

// ---------------------------------------------------------------------
void CAMERA::swap(unsigned int i, unsigned int j)
{
	if (i==j){}
	else{
		Point2f tmp;
		
		tmp 	= mc[i];
		mc[i] 	= mc[j];
		mc[j]	= tmp;
	}
}

// ---------------------------------------------------------------------
double CAMERA::square_dist(Point2f p, Point2f q)
{
	return ( (p.x-q.x) * (p.x-q.x) + (p.y-q.y) * (p.y-q.y) );
}


// ---------------------------------------------------------------------
double CAMERA::cross2d(Point2f p, Point2f q)
{
	return ( (p.x * q.y) - (p.y - q.x) );
}

// ---------------------------------------------------------------------
// Get current date/time, format is YY-MM-DD.HH:mm:ss
// http://stackoverflow.com/questions/997946/how-to-get-current-time-and-date-in-c
// http://en.cppreference.com/w/cpp/chrono/c/strftime
const string CAMERA::currentDateTime(void)
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

void CAMERA::EnableDataSaving(void)
{
	save_flag = true; 
	snprintf(rslt_file_name, FILENAME_SIZE, "../results/TestCAM_%d__%s.dat",test_number,currentDateTime().c_str());
	f = fopen(rslt_file_name,"w");
	if (f == NULL)
	{
		printf("Error opening results file!\n");
		exit(1);
	}
}


//----------------------------------------------------------------------
void CAMERA::SaveData(void)
{
	if (save_flag)
	{
		fprintf(f,"%.3f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t%3.6f\t\n",
			1000.0 * getTimeElapsed(time_elapsed), 
			rel_state[0],
			rel_state[1],
			rel_state[2],
			rel_state[3],//*180.0/3.14,
			rel_state[4],//*180.0/3.14,
			rel_state[5]//*180.0/3.14
			);	
	}
	/*
	ser->sendCharToArduino('c');
	float anglesToArd[3];
	for(int i = 0; i<3; i++) {
		anglesToArd[i] = rel_state[i+3]*180.0/3.14;
	}
	ser->sendDataToArduino(anglesToArd);
	*/
	
	//Calcolo il quaternione relativo agli angoli dati
	if(*camCommunicationON) {
		float cPhi = cos(rel_state[PHI] * 0.5);
		float sPhi = sin(rel_state[PHI] * 0.5);
		float cTheta = cos(rel_state[THETA] * 0.5);
		float sTheta = sin(rel_state[THETA] * 0.5);
		float cPsi = cos(rel_state[PSI] * 0.5);
		float sPsi = sin(rel_state[PSI] * 0.5);

		float quaternion[4];
		quaternion[0] = cPhi * cTheta * cPsi + sPhi * sTheta * sPsi;
		quaternion[1] = sPhi * cTheta * cPsi - cPhi * sTheta * sPsi;
		quaternion[2] = cPhi * sTheta * cPsi + sPhi * cTheta * sPsi;
		quaternion[3] = cPhi * cTheta * sPsi - sPhi * sTheta * cPsi;
		
		filt->measurementUpdate(quaternion);
	}
}

// --------------------------0-------------------------------------------
bool CAMERA::isOutOfFrame(void)
{
	
	for (int i=0; i<5; i++)
	{
		if (mc[i].x < 10.0 || mc[i].x > 630.0 || mc[i].y < 10.0 || mc[i].y > 470.0 )
			return true;
	}
	return false;
}



// ---------------------------------------------------------------------
void CAMERA::Run(void)
{
	raspicam::RaspiCam_Cv cam; 

	
	// INIT
	
	// set camera properties
	cam.set(CV_CAP_PROP_FRAME_WIDTH, frameWidth);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, frameHeight);
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_EXPOSURE, expTime);
	
	src.create((int)frameWidth, (int)frameHeight, CV_8UC1);
	dst.create((int)frameWidth, (int)frameHeight, CV_8UC1);
	
	#ifdef DEBUG
		cout << "Acquisition info:" << endl 
		<< "img width = " << cam.get(CV_CAP_PROP_FRAME_WIDTH) << endl 
		<< "img height = " << cam.get(CV_CAP_PROP_FRAME_HEIGHT) << endl 
		<< "exposure time = " << cam.get(CV_CAP_PROP_EXPOSURE) << endl
		<< "fps = " << cam.get(CV_CAP_PROP_FPS) << endl
		<< endl ;
	#endif
	
	#ifdef VISUALIZE
		rgb.create((int)frameWidth, (int)frameHeight, CV_8UC1);
	
		//cv::namedWindow("rgb", CV_WINDOW_AUTOSIZE); // decommentare questa all'occorrenza
		//cv::namedWindow("dst", CV_WINDOW_AUTOSIZE);
		
		clr.push_back(cv::Scalar(255,0,0));
		clr.push_back(cv::Scalar(0,255,0));
		clr.push_back(cv::Scalar(0,0,255));
		clr.push_back(cv::Scalar(0,255,255));
		clr.push_back(cv::Scalar(229,204,255));
	#endif

	// reserve space for saving blob coordinates
	mu.reserve(N_BLOBS);
	mc.reserve(N_BLOBS);
	mc_prev.reserve(N_BLOBS);

	loadCameraMatrix( CM, invCM, (int)frameWidth, (int)frameHeight );
	loadWorldPts( worldPts );
		
	isFirstFrame = true;
	
	EnableDataSaving();
	
	
	// PROCESS
	
	#ifdef DEBUG_PERFORMANCE
		int counter = 0;
	#endif
		
	if(!cam.open()) 
	{
		cerr << "Can't open camera!" << endl;
	}
    else
    {		
		
		gettimeofday(&start, NULL);
		gettimeofday(&curr_t, NULL);
		timersub(&curr_t,&start,&time_elapsed);
		
		while( (getTimeElapsed(time_elapsed) < test_duration) )
		{	
			#ifdef DEBUG_PERFORMANCE
				counter++;
			#endif
			
			cam.grab();
			cam.retrieve(src);
									
			#ifdef VISUALIZE
				cv::cvtColor(src, rgb, cv::COLOR_GRAY2RGB);
			#endif
			
			cv::threshold( src, dst, threshold_value, max_BINARY_value, 
				threshold_type );
			
			//imwrite("prova.png",dst);
			//waitKey();
			
			cv::findContours(dst, contours, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);
			cout << "cs: " << contours.size() << endl;
			
			if( contours.size() == N_BLOBS )
			{
				mc.erase( mc.begin(), mc.end());
				for (size_t i=0; i<contours.size(); i++)
				{
					mu[i] = cv::moments( contours[i], false );
					cout << "Sono qui " << i << endl;
					
					mc.push_back( cv::Point2f( static_cast<float>(mu[i].m10/mu[i].m00),
										static_cast<float>(mu[i].m01/mu[i].m00)) );

					cout << i << "\t" << mu[i].m00 << "\t" << mu[i].m01 << "\t" << mu[i].m10 << "\t" << mu[i].m11 << endl;
				}
				
				//cout << "mc.size() = " << mc.size() << endl;
				//cout << "mc.empty() = " << mc.empty() << endl;
				//cout << "mc = " << mc << endl;
				
				vector<Point2f> mc_raw(mc.size());
				mc_raw = mc;
				cout << "mc_raw = " << mc_raw << endl;
				
				
				// controlla se stiamo uscendo dall'immagine con i blob
				if (!isOutOfFrame()){
				
					#ifdef UNDISTORT_
					Mat mcMat(mc.size(),1,CV_32FC2);
					Mat mcRawMat(mc.size(),1,CV_32FC2);
					
					//cout << mcMat.rows << " x " << mcMat.cols << endl;
					
					for (int i=0; i < mcRawMat.rows; i++){
						mcRawMat.at<Point2f>(i) = mc_raw[i];
					}
					
					//cout << endl << "CMwrapped = " << CMwrapped << endl;
					//cout << endl << "dist_coeff = " << dist_coeff << endl;
					
					//cout << "mcRawMat = " << mcRawMat << endl;
					//cout << "===============================================" << endl;
										
					undistortPoints(mcRawMat,mcMat,CMwrapped,dist_coeff);
									
					//cout << "mcMat = " << mcMat << endl;
					//cout << "===============================================" << endl;
									
									
					for (int i=0; i < 5; i++){
						mc[i].x = mcMat.at<Point2f>(i).x * CM[0][0] + CM[0][2];
						mc[i].y = mcMat.at<Point2f>(i).y * CM[1][1] + CM[1][2];
					}

					cout << "after undistort" << endl << mc << endl;

					#endif // UNDISTORT_
					
					//cout << "mc = " << mc << endl;
					//cout << "===============================================" << endl;
					 
					 // /*void cv::undistortPoints 	( 	InputArray  	src,
						//OutputArray  	dst,
						//InputArray  	cameraMatrix,
						//InputArray  	distCoeffs,
						//InputArray  	R = noArray(),
						//InputArray  	P = noArray() 
					//) 		*/

					
					
					#ifdef DEBUG
					for (size_t i=0; i<contours.size(); i++)
						cout << "feat[" << i << "] = " << mc[i].x << " " << mc[i].y << endl;
					#endif	
					
					sortBlobs(); // rearrange blob order to match the correct listing
					
										cout << "COCAINE" << endl;					
					
					#ifdef VISUALIZE
						for (size_t i=0; i<contours.size(); i++){
							cv::circle(rgb, mc[i], 3, clr[i], -1);
							cv::drawContours( rgb, contours, i, clr[i], 3 ,0 );
						}
						imshow("rgb", rgb);
						waitKey(1);
					#endif //VISUALIZE
					
										cout << "HEROINE" << endl;

					
					#ifdef DEBUG
					for (size_t i=0; i<contours.size(); i++)
						cout << "feat[" << i << "] = " << mc[i].x << " " << mc[i].y << endl;
					#endif
					
					getFeatUnitVec( featureUnitVec );
					
					double err_param = 1001.0;
					double err = 10000.0;
					
					int cs = contours.size();
					TooN::Matrix<4> intermediate_sol;
					
					for (int count=0; count < cs ; count++)
					{
						TooN::Matrix<3> fMat, wMat;
						
						fMat[0] = featureUnitVec[(count)%cs];
						fMat[1] = featureUnitVec[(count+1)%cs];
						fMat[2] = featureUnitVec[(count+2)%cs];
						
						TooN::Matrix<2,3> chk_f_pts;
						chk_f_pts[0] = TooN::makeVector(mc[(count+3)%cs].x, mc[(count+3)%cs].y, 1.0);
						chk_f_pts[1] = TooN::makeVector(mc[(count+4)%cs].x, mc[(count+4)%cs].y, 1.0);
						
						wMat[0] = worldPts[(count)%cs];
						wMat[1] = worldPts[(count+1)%cs];
						wMat[2] = worldPts[(count+2)%cs];
						
						TooN::Matrix<2,3> chk_w_pts;
						chk_w_pts[0] = worldPts[(count+3)%cs];
						chk_w_pts[1] = worldPts[(count+4)%cs];
																
						p3p.computePoses( fMat.T(), wMat.T(), p3pSol );		
								
						chooseP3pSol(chk_w_pts, chk_f_pts, err );

						//#ifdef DEBUG
							cout << count << "\terr = " << err << "\terr_param = " << err_param << endl;
						//#endif
						
						if (err < err_param) 
						{							
							intermediate_sol = sol;
							err_param = err;
						}
						//else 
							//cerr << "VAFFANCULO MAZ!" << endl;	
					}
					//cout << getTimeElapsed(time_elapsed) << "chosenP3P:\terr_param = " << err_param << endl;
					if(err_param < threshold_error)	 {
						
						sol = intermediate_sol;
						
						//cout << "=======================================" << endl;
						//cout << sol << endl;
						//cout << "=======================================" << endl;
						
						setRelState(); // store relative state vector
						
						
						//setRelativeAttitudeInIMUFrame();
						
						
						SaveData();
					}
					else {cout << "Soluzione saltata, err: " << err_param << endl;}
					
					// store current blob position in mc_prev to be used in next iteration
					for(unsigned int i=0; i<N_BLOBS; i++)
						mc_prev[i] = mc[i];
				}
				else{ cout << "OOF!\n";}  //isOutOfFrame
				
			}
					
			//update condition to stop loop
			gettimeofday(&curr_t, NULL);
			timersub(&curr_t,&start,&time_elapsed);
		}
		
		fclose(f);
		//cam.release();
		
	}	
	#ifdef DEBUG_PERFORMANCE		
		fprintf(stdout,"number of iterations = %d\n", counter);
		fprintf(stdout,"fps = %.3f\n", (double)counter/getTimeElapsed(time_elapsed));
	#endif
	
	return;
	//exit(0);
}




