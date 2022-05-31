#####################################
cmake_minimum_required (VERSION 2.8) 
project (pacman)
							
set(CMAKE_CXX_FLAGS "-std=c++11 -g -lc -O0 -Wall -pthread")

add_executable (imu_upd ../main.cpp
						# include/myKalmanFilter.cpp
						# include/serialCommArdRasp.cpp
						# include/P3p.cpp
						# include/pacman_raspi.cpp
						../pacman_imu.hpp
						# include/global.h
						)  
target_link_libraries (imu_upd 	${CMAKE_THREAD_LIBS_INIT}
											-lphidget22
											-lpthread
											)
#####################################
 
