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
CMAKE_SOURCE_DIR = /home/m/ws_orb2/src/VISO2+

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/m/ws_orb2/src/VISO2+/bin

# Include any dependencies generated for this target.
include CMakeFiles/viso2_plus.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/viso2_plus.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/viso2_plus.dir/flags.make

CMakeFiles/viso2_plus.dir/src/demo.cpp.o: CMakeFiles/viso2_plus.dir/flags.make
CMakeFiles/viso2_plus.dir/src/demo.cpp.o: ../src/demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/m/ws_orb2/src/VISO2+/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/viso2_plus.dir/src/demo.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/viso2_plus.dir/src/demo.cpp.o -c /home/m/ws_orb2/src/VISO2+/src/demo.cpp

CMakeFiles/viso2_plus.dir/src/demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/viso2_plus.dir/src/demo.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/m/ws_orb2/src/VISO2+/src/demo.cpp > CMakeFiles/viso2_plus.dir/src/demo.cpp.i

CMakeFiles/viso2_plus.dir/src/demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/viso2_plus.dir/src/demo.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/m/ws_orb2/src/VISO2+/src/demo.cpp -o CMakeFiles/viso2_plus.dir/src/demo.cpp.s

CMakeFiles/viso2_plus.dir/src/demo.cpp.o.requires:

.PHONY : CMakeFiles/viso2_plus.dir/src/demo.cpp.o.requires

CMakeFiles/viso2_plus.dir/src/demo.cpp.o.provides: CMakeFiles/viso2_plus.dir/src/demo.cpp.o.requires
	$(MAKE) -f CMakeFiles/viso2_plus.dir/build.make CMakeFiles/viso2_plus.dir/src/demo.cpp.o.provides.build
.PHONY : CMakeFiles/viso2_plus.dir/src/demo.cpp.o.provides

CMakeFiles/viso2_plus.dir/src/demo.cpp.o.provides.build: CMakeFiles/viso2_plus.dir/src/demo.cpp.o


# Object files for target viso2_plus
viso2_plus_OBJECTS = \
"CMakeFiles/viso2_plus.dir/src/demo.cpp.o"

# External object files for target viso2_plus
viso2_plus_EXTERNAL_OBJECTS =

viso2_plus: CMakeFiles/viso2_plus.dir/src/demo.cpp.o
viso2_plus: CMakeFiles/viso2_plus.dir/build.make
viso2_plus: ../lib/libVISO2_PLUS.so
viso2_plus: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_superres3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_face3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_plot3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_reg3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_viz3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_text3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_shape3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_video3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_flann3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_ml3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_photo3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.2.0
viso2_plus: /opt/ros/kinetic/lib/libopencv_core3.so.3.2.0
viso2_plus: /home/m/Sophus/build/libSophus.so
viso2_plus: /usr/local/lib/libpangolin.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libGLU.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libGL.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libGLEW.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libSM.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libICE.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libX11.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libXext.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libpython2.7.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libavcodec.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libavformat.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libavutil.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libswscale.so
viso2_plus: /usr/lib/libOpenNI.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libpng.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libz.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libjpeg.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libtiff.so
viso2_plus: /usr/lib/x86_64-linux-gnu/libIlmImf.so
viso2_plus: ../Thirdparty/DBoW2/lib/libDBoW2.so
viso2_plus: ../Thirdparty/g2o/lib/libg2o.so
viso2_plus: CMakeFiles/viso2_plus.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/m/ws_orb2/src/VISO2+/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable viso2_plus"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/viso2_plus.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/viso2_plus.dir/build: viso2_plus

.PHONY : CMakeFiles/viso2_plus.dir/build

CMakeFiles/viso2_plus.dir/requires: CMakeFiles/viso2_plus.dir/src/demo.cpp.o.requires

.PHONY : CMakeFiles/viso2_plus.dir/requires

CMakeFiles/viso2_plus.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/viso2_plus.dir/cmake_clean.cmake
.PHONY : CMakeFiles/viso2_plus.dir/clean

CMakeFiles/viso2_plus.dir/depend:
	cd /home/m/ws_orb2/src/VISO2+/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/m/ws_orb2/src/VISO2+ /home/m/ws_orb2/src/VISO2+ /home/m/ws_orb2/src/VISO2+/bin /home/m/ws_orb2/src/VISO2+/bin /home/m/ws_orb2/src/VISO2+/bin/CMakeFiles/viso2_plus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/viso2_plus.dir/depend

