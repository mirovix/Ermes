#####################################
cmake_minimum_required (VERSION 2.8) 
project (pacman)

set(raspicam_DIR "/usr/local/lib/cmake")

#add_definitions(-DBOOST_LOG_DYN_LINK=1)

find_package(raspicam REQUIRED)
find_package(OpenCV)
							
set(CMAKE_CXX_FLAGS "-std=c++11 -g -lc -O0 -Wall -pthread")

IF  ( OpenCV_FOUND AND raspicam_CV_FOUND)
	MESSAGE(STATUS "OPENCV AND RASPICAM ---> FOUND!")
	
	include_directories( ./include )
	add_executable (pacman_exe src/main.cpp	
						include/P3p.cpp
						include/pacman_raspi.cpp
						include/pacman_imu.hpp
						include/global.h
						)  
	target_link_libraries (pacman_exe ${raspicam_CV_LIBS} 
											${CMAKE_THREAD_LIBS_INIT}
											-lphidget22)

ELSE()
	MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()
#####################################
 
