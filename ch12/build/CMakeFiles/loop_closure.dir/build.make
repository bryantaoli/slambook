# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ht/slambook/ch12

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ht/slambook/ch12/build

# Include any dependencies generated for this target.
include CMakeFiles/loop_closure.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/loop_closure.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/loop_closure.dir/flags.make

CMakeFiles/loop_closure.dir/loop_closure.cpp.o: CMakeFiles/loop_closure.dir/flags.make
CMakeFiles/loop_closure.dir/loop_closure.cpp.o: ../loop_closure.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ht/slambook/ch12/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/loop_closure.dir/loop_closure.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/loop_closure.dir/loop_closure.cpp.o -c /home/ht/slambook/ch12/loop_closure.cpp

CMakeFiles/loop_closure.dir/loop_closure.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/loop_closure.dir/loop_closure.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ht/slambook/ch12/loop_closure.cpp > CMakeFiles/loop_closure.dir/loop_closure.cpp.i

CMakeFiles/loop_closure.dir/loop_closure.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/loop_closure.dir/loop_closure.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ht/slambook/ch12/loop_closure.cpp -o CMakeFiles/loop_closure.dir/loop_closure.cpp.s

# Object files for target loop_closure
loop_closure_OBJECTS = \
"CMakeFiles/loop_closure.dir/loop_closure.cpp.o"

# External object files for target loop_closure
loop_closure_EXTERNAL_OBJECTS =

loop_closure: CMakeFiles/loop_closure.dir/loop_closure.cpp.o
loop_closure: CMakeFiles/loop_closure.dir/build.make
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
loop_closure: /usr/local/lib/libDBoW3.a
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
loop_closure: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
loop_closure: CMakeFiles/loop_closure.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ht/slambook/ch12/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable loop_closure"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/loop_closure.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/loop_closure.dir/build: loop_closure

.PHONY : CMakeFiles/loop_closure.dir/build

CMakeFiles/loop_closure.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/loop_closure.dir/cmake_clean.cmake
.PHONY : CMakeFiles/loop_closure.dir/clean

CMakeFiles/loop_closure.dir/depend:
	cd /home/ht/slambook/ch12/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ht/slambook/ch12 /home/ht/slambook/ch12 /home/ht/slambook/ch12/build /home/ht/slambook/ch12/build /home/ht/slambook/ch12/build/CMakeFiles/loop_closure.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/loop_closure.dir/depend

