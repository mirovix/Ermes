# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /media/pi/USB_CUBE1/pacman_test_reinit_pattern

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /media/pi/USB_CUBE1/pacman_test_reinit_pattern/build

# Include any dependencies generated for this target.
include CMakeFiles/pacman_exe.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pacman_exe.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pacman_exe.dir/flags.make

CMakeFiles/pacman_exe.dir/src/main.cpp.o: CMakeFiles/pacman_exe.dir/flags.make
CMakeFiles/pacman_exe.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pacman_exe.dir/src/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pacman_exe.dir/src/main.cpp.o -c /media/pi/USB_CUBE1/pacman_test_reinit_pattern/src/main.cpp

CMakeFiles/pacman_exe.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pacman_exe.dir/src/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/pi/USB_CUBE1/pacman_test_reinit_pattern/src/main.cpp > CMakeFiles/pacman_exe.dir/src/main.cpp.i

CMakeFiles/pacman_exe.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pacman_exe.dir/src/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/pi/USB_CUBE1/pacman_test_reinit_pattern/src/main.cpp -o CMakeFiles/pacman_exe.dir/src/main.cpp.s

CMakeFiles/pacman_exe.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/pacman_exe.dir/src/main.cpp.o.requires

CMakeFiles/pacman_exe.dir/src/main.cpp.o.provides: CMakeFiles/pacman_exe.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/pacman_exe.dir/build.make CMakeFiles/pacman_exe.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/pacman_exe.dir/src/main.cpp.o.provides

CMakeFiles/pacman_exe.dir/src/main.cpp.o.provides.build: CMakeFiles/pacman_exe.dir/src/main.cpp.o


CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o: CMakeFiles/pacman_exe.dir/flags.make
CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o: ../include/myKalmanFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o -c /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/myKalmanFilter.cpp

CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/myKalmanFilter.cpp > CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.i

CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/myKalmanFilter.cpp -o CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.s

CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.requires:

.PHONY : CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.requires

CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.provides: CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/pacman_exe.dir/build.make CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.provides.build
.PHONY : CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.provides

CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.provides.build: CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o


CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o: CMakeFiles/pacman_exe.dir/flags.make
CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o: ../include/serialCommArdRasp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o -c /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/serialCommArdRasp.cpp

CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/serialCommArdRasp.cpp > CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.i

CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/serialCommArdRasp.cpp -o CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.s

CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.requires:

.PHONY : CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.requires

CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.provides: CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.requires
	$(MAKE) -f CMakeFiles/pacman_exe.dir/build.make CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.provides.build
.PHONY : CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.provides

CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.provides.build: CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o


CMakeFiles/pacman_exe.dir/include/P3p.cpp.o: CMakeFiles/pacman_exe.dir/flags.make
CMakeFiles/pacman_exe.dir/include/P3p.cpp.o: ../include/P3p.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/pacman_exe.dir/include/P3p.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pacman_exe.dir/include/P3p.cpp.o -c /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/P3p.cpp

CMakeFiles/pacman_exe.dir/include/P3p.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pacman_exe.dir/include/P3p.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/P3p.cpp > CMakeFiles/pacman_exe.dir/include/P3p.cpp.i

CMakeFiles/pacman_exe.dir/include/P3p.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pacman_exe.dir/include/P3p.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/P3p.cpp -o CMakeFiles/pacman_exe.dir/include/P3p.cpp.s

CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.requires:

.PHONY : CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.requires

CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.provides: CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.requires
	$(MAKE) -f CMakeFiles/pacman_exe.dir/build.make CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.provides.build
.PHONY : CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.provides

CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.provides.build: CMakeFiles/pacman_exe.dir/include/P3p.cpp.o


CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o: CMakeFiles/pacman_exe.dir/flags.make
CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o: ../include/pacman_raspi.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o -c /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/pacman_raspi.cpp

CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/pacman_raspi.cpp > CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.i

CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /media/pi/USB_CUBE1/pacman_test_reinit_pattern/include/pacman_raspi.cpp -o CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.s

CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.requires:

.PHONY : CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.requires

CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.provides: CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.requires
	$(MAKE) -f CMakeFiles/pacman_exe.dir/build.make CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.provides.build
.PHONY : CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.provides

CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.provides.build: CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o


# Object files for target pacman_exe
pacman_exe_OBJECTS = \
"CMakeFiles/pacman_exe.dir/src/main.cpp.o" \
"CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o" \
"CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o" \
"CMakeFiles/pacman_exe.dir/include/P3p.cpp.o" \
"CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o"

# External object files for target pacman_exe
pacman_exe_EXTERNAL_OBJECTS =

pacman_exe: CMakeFiles/pacman_exe.dir/src/main.cpp.o
pacman_exe: CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o
pacman_exe: CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o
pacman_exe: CMakeFiles/pacman_exe.dir/include/P3p.cpp.o
pacman_exe: CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o
pacman_exe: CMakeFiles/pacman_exe.dir/build.make
pacman_exe: /opt/vc/lib/libmmal_core.so
pacman_exe: /opt/vc/lib/libmmal_util.so
pacman_exe: /opt/vc/lib/libmmal.so
pacman_exe: /usr/local/lib/libopencv_videostab.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_superres.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_stitching.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_shape.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_photo.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_objdetect.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_calib3d.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_features2d.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_ml.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_highgui.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_videoio.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_imgcodecs.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_flann.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_video.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_imgproc.so.3.1.0
pacman_exe: /usr/local/lib/libopencv_core.so.3.1.0
pacman_exe: CMakeFiles/pacman_exe.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable pacman_exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pacman_exe.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pacman_exe.dir/build: pacman_exe

.PHONY : CMakeFiles/pacman_exe.dir/build

CMakeFiles/pacman_exe.dir/requires: CMakeFiles/pacman_exe.dir/src/main.cpp.o.requires
CMakeFiles/pacman_exe.dir/requires: CMakeFiles/pacman_exe.dir/include/myKalmanFilter.cpp.o.requires
CMakeFiles/pacman_exe.dir/requires: CMakeFiles/pacman_exe.dir/include/serialCommArdRasp.cpp.o.requires
CMakeFiles/pacman_exe.dir/requires: CMakeFiles/pacman_exe.dir/include/P3p.cpp.o.requires
CMakeFiles/pacman_exe.dir/requires: CMakeFiles/pacman_exe.dir/include/pacman_raspi.cpp.o.requires

.PHONY : CMakeFiles/pacman_exe.dir/requires

CMakeFiles/pacman_exe.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pacman_exe.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pacman_exe.dir/clean

CMakeFiles/pacman_exe.dir/depend:
	cd /media/pi/USB_CUBE1/pacman_test_reinit_pattern/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /media/pi/USB_CUBE1/pacman_test_reinit_pattern /media/pi/USB_CUBE1/pacman_test_reinit_pattern /media/pi/USB_CUBE1/pacman_test_reinit_pattern/build /media/pi/USB_CUBE1/pacman_test_reinit_pattern/build /media/pi/USB_CUBE1/pacman_test_reinit_pattern/build/CMakeFiles/pacman_exe.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pacman_exe.dir/depend

