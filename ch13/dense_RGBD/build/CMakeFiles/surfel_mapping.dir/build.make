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
CMAKE_SOURCE_DIR = /home/ht/slambook/ch13/dense_RGBD

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ht/slambook/ch13/dense_RGBD/build

# Include any dependencies generated for this target.
include CMakeFiles/surfel_mapping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/surfel_mapping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/surfel_mapping.dir/flags.make

CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.o: CMakeFiles/surfel_mapping.dir/flags.make
CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.o: ../surfel_mapping.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ht/slambook/ch13/dense_RGBD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.o -c /home/ht/slambook/ch13/dense_RGBD/surfel_mapping.cpp

CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ht/slambook/ch13/dense_RGBD/surfel_mapping.cpp > CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.i

CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ht/slambook/ch13/dense_RGBD/surfel_mapping.cpp -o CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.s

# Object files for target surfel_mapping
surfel_mapping_OBJECTS = \
"CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.o"

# External object files for target surfel_mapping
surfel_mapping_EXTERNAL_OBJECTS =

surfel_mapping: CMakeFiles/surfel_mapping.dir/surfel_mapping.cpp.o
surfel_mapping: CMakeFiles/surfel_mapping.dir/build.make
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
surfel_mapping: /usr/local/lib/libpcl_apps.so
surfel_mapping: /usr/local/lib/libpcl_outofcore.so
surfel_mapping: /usr/local/lib/libpcl_people.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libboost_system.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libboost_regex.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libqhull.so
surfel_mapping: /usr/lib/libOpenNI.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libfreetype.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libz.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libjpeg.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libpng.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libtiff.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libjsoncpp.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingQt-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL-6.2.so.6.2.0
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
surfel_mapping: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
surfel_mapping: /usr/local/lib/libpcl_surface.so
surfel_mapping: /usr/local/lib/libpcl_keypoints.so
surfel_mapping: /usr/local/lib/libpcl_tracking.so
surfel_mapping: /usr/local/lib/libpcl_recognition.so
surfel_mapping: /usr/local/lib/libpcl_registration.so
surfel_mapping: /usr/local/lib/libpcl_stereo.so
surfel_mapping: /usr/local/lib/libpcl_segmentation.so
surfel_mapping: /usr/local/lib/libpcl_features.so
surfel_mapping: /usr/local/lib/libpcl_filters.so
surfel_mapping: /usr/local/lib/libpcl_sample_consensus.so
surfel_mapping: /usr/local/lib/libpcl_ml.so
surfel_mapping: /usr/local/lib/libpcl_visualization.so
surfel_mapping: /usr/local/lib/libpcl_search.so
surfel_mapping: /usr/local/lib/libpcl_kdtree.so
surfel_mapping: /usr/local/lib/libpcl_io.so
surfel_mapping: /usr/local/lib/libpcl_octree.so
surfel_mapping: /usr/local/lib/libpcl_common.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersTexture-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkIOImage-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkIOCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkmetaio-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libz.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libGLU.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libSM.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libICE.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libX11.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libXext.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libXt.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingLabel-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkalglib-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtksys-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libvtkftgl-6.2.so.6.2.0
surfel_mapping: /usr/lib/x86_64-linux-gnu/libfreetype.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libGL.so
surfel_mapping: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.5.1
surfel_mapping: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.5.1
surfel_mapping: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.5.1
surfel_mapping: CMakeFiles/surfel_mapping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ht/slambook/ch13/dense_RGBD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable surfel_mapping"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/surfel_mapping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/surfel_mapping.dir/build: surfel_mapping

.PHONY : CMakeFiles/surfel_mapping.dir/build

CMakeFiles/surfel_mapping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/surfel_mapping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/surfel_mapping.dir/clean

CMakeFiles/surfel_mapping.dir/depend:
	cd /home/ht/slambook/ch13/dense_RGBD/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ht/slambook/ch13/dense_RGBD /home/ht/slambook/ch13/dense_RGBD /home/ht/slambook/ch13/dense_RGBD/build /home/ht/slambook/ch13/dense_RGBD/build /home/ht/slambook/ch13/dense_RGBD/build/CMakeFiles/surfel_mapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/surfel_mapping.dir/depend

