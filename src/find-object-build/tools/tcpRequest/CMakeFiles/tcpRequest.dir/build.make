# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src/find-object

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/src/find-object-build

# Include any dependencies generated for this target.
include tools/tcpRequest/CMakeFiles/tcpRequest.dir/depend.make

# Include the progress variables for this target.
include tools/tcpRequest/CMakeFiles/tcpRequest.dir/progress.make

# Include the compile flags for this target's objects.
include tools/tcpRequest/CMakeFiles/tcpRequest.dir/flags.make

tools/tcpRequest/moc_TcpResponse.cpp: /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/TcpResponse.h
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ubuntu/catkin_ws/src/find-object-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating moc_TcpResponse.cpp"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/lib/aarch64-linux-gnu/qt5/bin/moc @/home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest/moc_TcpResponse.cpp_parameters

tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o: tools/tcpRequest/CMakeFiles/tcpRequest.dir/flags.make
tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o: /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/TcpResponse.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/find-object-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o -c /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/TcpResponse.cpp

tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tcpRequest.dir/TcpResponse.cpp.i"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/TcpResponse.cpp > CMakeFiles/tcpRequest.dir/TcpResponse.cpp.i

tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tcpRequest.dir/TcpResponse.cpp.s"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/TcpResponse.cpp -o CMakeFiles/tcpRequest.dir/TcpResponse.cpp.s

tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.requires:

.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.requires

tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.provides: tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.requires
	$(MAKE) -f tools/tcpRequest/CMakeFiles/tcpRequest.dir/build.make tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.provides.build
.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.provides

tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.provides.build: tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o


tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o: tools/tcpRequest/CMakeFiles/tcpRequest.dir/flags.make
tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o: /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/find-object-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tcpRequest.dir/main.cpp.o -c /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/main.cpp

tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tcpRequest.dir/main.cpp.i"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/main.cpp > CMakeFiles/tcpRequest.dir/main.cpp.i

tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tcpRequest.dir/main.cpp.s"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest/main.cpp -o CMakeFiles/tcpRequest.dir/main.cpp.s

tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.requires:

.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.requires

tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.provides: tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.requires
	$(MAKE) -f tools/tcpRequest/CMakeFiles/tcpRequest.dir/build.make tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.provides.build
.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.provides

tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.provides.build: tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o


tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o: tools/tcpRequest/CMakeFiles/tcpRequest.dir/flags.make
tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o: tools/tcpRequest/moc_TcpResponse.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/catkin_ws/src/find-object-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o -c /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest/moc_TcpResponse.cpp

tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.i"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest/moc_TcpResponse.cpp > CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.i

tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.s"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest/moc_TcpResponse.cpp -o CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.s

tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.requires:

.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.requires

tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.provides: tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.requires
	$(MAKE) -f tools/tcpRequest/CMakeFiles/tcpRequest.dir/build.make tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.provides.build
.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.provides

tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.provides.build: tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o


# Object files for target tcpRequest
tcpRequest_OBJECTS = \
"CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o" \
"CMakeFiles/tcpRequest.dir/main.cpp.o" \
"CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o"

# External object files for target tcpRequest
tcpRequest_EXTERNAL_OBJECTS =

/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: tools/tcpRequest/CMakeFiles/tcpRequest.dir/build.make
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /home/ubuntu/catkin_ws/src/find-object/bin/libfind_object.so
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_reg3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_plot3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_viz3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_superres3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_photo3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /usr/lib/aarch64-linux-gnu/libQt5Network.so.5.5.1
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /usr/lib/aarch64-linux-gnu/libQt5PrintSupport.so.5.5.1
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_text3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_face3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_shape3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_video3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_ml3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_flann3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /opt/ros/kinetic/lib/libopencv_core3.so.3.1.0
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /usr/lib/aarch64-linux-gnu/libQt5Widgets.so.5.5.1
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /usr/lib/aarch64-linux-gnu/libQt5Gui.so.5.5.1
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: /usr/lib/aarch64-linux-gnu/libQt5Core.so.5.5.1
/home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest: tools/tcpRequest/CMakeFiles/tcpRequest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/catkin_ws/src/find-object-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable /home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest"
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/tcpRequest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/tcpRequest/CMakeFiles/tcpRequest.dir/build: /home/ubuntu/catkin_ws/src/find-object/bin/find_object-tcpRequest

.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/build

tools/tcpRequest/CMakeFiles/tcpRequest.dir/requires: tools/tcpRequest/CMakeFiles/tcpRequest.dir/TcpResponse.cpp.o.requires
tools/tcpRequest/CMakeFiles/tcpRequest.dir/requires: tools/tcpRequest/CMakeFiles/tcpRequest.dir/main.cpp.o.requires
tools/tcpRequest/CMakeFiles/tcpRequest.dir/requires: tools/tcpRequest/CMakeFiles/tcpRequest.dir/moc_TcpResponse.cpp.o.requires

.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/requires

tools/tcpRequest/CMakeFiles/tcpRequest.dir/clean:
	cd /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest && $(CMAKE_COMMAND) -P CMakeFiles/tcpRequest.dir/cmake_clean.cmake
.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/clean

tools/tcpRequest/CMakeFiles/tcpRequest.dir/depend: tools/tcpRequest/moc_TcpResponse.cpp
	cd /home/ubuntu/catkin_ws/src/find-object-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src/find-object /home/ubuntu/catkin_ws/src/find-object/tools/tcpRequest /home/ubuntu/catkin_ws/src/find-object-build /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest /home/ubuntu/catkin_ws/src/find-object-build/tools/tcpRequest/CMakeFiles/tcpRequest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/tcpRequest/CMakeFiles/tcpRequest.dir/depend
