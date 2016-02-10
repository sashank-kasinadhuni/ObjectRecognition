# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build

# Include any dependencies generated for this target.
include CMakeFiles/histogram.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/histogram.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/histogram.dir/flags.make

CMakeFiles/histogram.dir/histogram.cpp.o: CMakeFiles/histogram.dir/flags.make
CMakeFiles/histogram.dir/histogram.cpp.o: ../histogram.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/histogram.dir/histogram.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/histogram.dir/histogram.cpp.o -c /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/histogram.cpp

CMakeFiles/histogram.dir/histogram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/histogram.dir/histogram.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/histogram.cpp > CMakeFiles/histogram.dir/histogram.cpp.i

CMakeFiles/histogram.dir/histogram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/histogram.dir/histogram.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/histogram.cpp -o CMakeFiles/histogram.dir/histogram.cpp.s

CMakeFiles/histogram.dir/histogram.cpp.o.requires:
.PHONY : CMakeFiles/histogram.dir/histogram.cpp.o.requires

CMakeFiles/histogram.dir/histogram.cpp.o.provides: CMakeFiles/histogram.dir/histogram.cpp.o.requires
	$(MAKE) -f CMakeFiles/histogram.dir/build.make CMakeFiles/histogram.dir/histogram.cpp.o.provides.build
.PHONY : CMakeFiles/histogram.dir/histogram.cpp.o.provides

CMakeFiles/histogram.dir/histogram.cpp.o.provides.build: CMakeFiles/histogram.dir/histogram.cpp.o

CMakeFiles/histogram.dir/planar_filtering.cpp.o: CMakeFiles/histogram.dir/flags.make
CMakeFiles/histogram.dir/planar_filtering.cpp.o: ../planar_filtering.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/histogram.dir/planar_filtering.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/histogram.dir/planar_filtering.cpp.o -c /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/planar_filtering.cpp

CMakeFiles/histogram.dir/planar_filtering.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/histogram.dir/planar_filtering.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/planar_filtering.cpp > CMakeFiles/histogram.dir/planar_filtering.cpp.i

CMakeFiles/histogram.dir/planar_filtering.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/histogram.dir/planar_filtering.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/planar_filtering.cpp -o CMakeFiles/histogram.dir/planar_filtering.cpp.s

CMakeFiles/histogram.dir/planar_filtering.cpp.o.requires:
.PHONY : CMakeFiles/histogram.dir/planar_filtering.cpp.o.requires

CMakeFiles/histogram.dir/planar_filtering.cpp.o.provides: CMakeFiles/histogram.dir/planar_filtering.cpp.o.requires
	$(MAKE) -f CMakeFiles/histogram.dir/build.make CMakeFiles/histogram.dir/planar_filtering.cpp.o.provides.build
.PHONY : CMakeFiles/histogram.dir/planar_filtering.cpp.o.provides

CMakeFiles/histogram.dir/planar_filtering.cpp.o.provides.build: CMakeFiles/histogram.dir/planar_filtering.cpp.o

CMakeFiles/histogram.dir/recognition_features.cpp.o: CMakeFiles/histogram.dir/flags.make
CMakeFiles/histogram.dir/recognition_features.cpp.o: ../recognition_features.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/histogram.dir/recognition_features.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/histogram.dir/recognition_features.cpp.o -c /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/recognition_features.cpp

CMakeFiles/histogram.dir/recognition_features.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/histogram.dir/recognition_features.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/recognition_features.cpp > CMakeFiles/histogram.dir/recognition_features.cpp.i

CMakeFiles/histogram.dir/recognition_features.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/histogram.dir/recognition_features.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/recognition_features.cpp -o CMakeFiles/histogram.dir/recognition_features.cpp.s

CMakeFiles/histogram.dir/recognition_features.cpp.o.requires:
.PHONY : CMakeFiles/histogram.dir/recognition_features.cpp.o.requires

CMakeFiles/histogram.dir/recognition_features.cpp.o.provides: CMakeFiles/histogram.dir/recognition_features.cpp.o.requires
	$(MAKE) -f CMakeFiles/histogram.dir/build.make CMakeFiles/histogram.dir/recognition_features.cpp.o.provides.build
.PHONY : CMakeFiles/histogram.dir/recognition_features.cpp.o.provides

CMakeFiles/histogram.dir/recognition_features.cpp.o.provides.build: CMakeFiles/histogram.dir/recognition_features.cpp.o

# Object files for target histogram
histogram_OBJECTS = \
"CMakeFiles/histogram.dir/histogram.cpp.o" \
"CMakeFiles/histogram.dir/planar_filtering.cpp.o" \
"CMakeFiles/histogram.dir/recognition_features.cpp.o"

# External object files for target histogram
histogram_EXTERNAL_OBJECTS =

histogram: CMakeFiles/histogram.dir/histogram.cpp.o
histogram: CMakeFiles/histogram.dir/planar_filtering.cpp.o
histogram: CMakeFiles/histogram.dir/recognition_features.cpp.o
histogram: CMakeFiles/histogram.dir/build.make
histogram: /usr/lib/x86_64-linux-gnu/libboost_system.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_thread.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
histogram: /usr/lib/x86_64-linux-gnu/libpthread.so
histogram: /usr/local/lib/libpcl_common.so
histogram: /usr/local/lib/libpcl_octree.so
histogram: /usr/lib/libOpenNI.so
histogram: /usr/lib/libOpenNI2.so
histogram: /usr/lib/libvtkCommon.so.5.8.0
histogram: /usr/lib/libvtkFiltering.so.5.8.0
histogram: /usr/lib/libvtkImaging.so.5.8.0
histogram: /usr/lib/libvtkGraphics.so.5.8.0
histogram: /usr/lib/libvtkGenericFiltering.so.5.8.0
histogram: /usr/lib/libvtkIO.so.5.8.0
histogram: /usr/lib/libvtkRendering.so.5.8.0
histogram: /usr/lib/libvtkVolumeRendering.so.5.8.0
histogram: /usr/lib/libvtkHybrid.so.5.8.0
histogram: /usr/lib/libvtkWidgets.so.5.8.0
histogram: /usr/lib/libvtkParallel.so.5.8.0
histogram: /usr/lib/libvtkInfovis.so.5.8.0
histogram: /usr/lib/libvtkGeovis.so.5.8.0
histogram: /usr/lib/libvtkViews.so.5.8.0
histogram: /usr/lib/libvtkCharts.so.5.8.0
histogram: /usr/local/lib/libpcl_io.so
histogram: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
histogram: /usr/local/lib/libpcl_kdtree.so
histogram: /usr/local/lib/libpcl_search.so
histogram: /usr/local/lib/libpcl_sample_consensus.so
histogram: /usr/local/lib/libpcl_filters.so
histogram: /usr/local/lib/libpcl_features.so
histogram: /usr/local/lib/libpcl_registration.so
histogram: /usr/local/lib/libpcl_recognition.so
histogram: /usr/local/lib/libpcl_keypoints.so
histogram: /usr/local/lib/libpcl_segmentation.so
histogram: /usr/local/lib/libpcl_visualization.so
histogram: /usr/local/lib/libpcl_outofcore.so
histogram: /usr/local/lib/libpcl_tracking.so
histogram: /usr/local/lib/libpcl_people.so
histogram: /usr/lib/x86_64-linux-gnu/libqhull.so
histogram: /usr/local/lib/libpcl_surface.so
histogram: /usr/local/lib/libpcl_apps.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_system.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_thread.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
histogram: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
histogram: /usr/lib/x86_64-linux-gnu/libpthread.so
histogram: /usr/lib/x86_64-linux-gnu/libqhull.so
histogram: /usr/lib/libOpenNI.so
histogram: /usr/lib/libOpenNI2.so
histogram: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
histogram: /usr/lib/libvtkCommon.so.5.8.0
histogram: /usr/lib/libvtkFiltering.so.5.8.0
histogram: /usr/lib/libvtkImaging.so.5.8.0
histogram: /usr/lib/libvtkGraphics.so.5.8.0
histogram: /usr/lib/libvtkGenericFiltering.so.5.8.0
histogram: /usr/lib/libvtkIO.so.5.8.0
histogram: /usr/lib/libvtkRendering.so.5.8.0
histogram: /usr/lib/libvtkVolumeRendering.so.5.8.0
histogram: /usr/lib/libvtkHybrid.so.5.8.0
histogram: /usr/lib/libvtkWidgets.so.5.8.0
histogram: /usr/lib/libvtkParallel.so.5.8.0
histogram: /usr/lib/libvtkInfovis.so.5.8.0
histogram: /usr/lib/libvtkGeovis.so.5.8.0
histogram: /usr/lib/libvtkViews.so.5.8.0
histogram: /usr/lib/libvtkCharts.so.5.8.0
histogram: /usr/local/lib/libpcl_common.so
histogram: /usr/local/lib/libpcl_octree.so
histogram: /usr/local/lib/libpcl_io.so
histogram: /usr/local/lib/libpcl_kdtree.so
histogram: /usr/local/lib/libpcl_search.so
histogram: /usr/local/lib/libpcl_sample_consensus.so
histogram: /usr/local/lib/libpcl_filters.so
histogram: /usr/local/lib/libpcl_features.so
histogram: /usr/local/lib/libpcl_registration.so
histogram: /usr/local/lib/libpcl_recognition.so
histogram: /usr/local/lib/libpcl_keypoints.so
histogram: /usr/local/lib/libpcl_segmentation.so
histogram: /usr/local/lib/libpcl_visualization.so
histogram: /usr/local/lib/libpcl_outofcore.so
histogram: /usr/local/lib/libpcl_tracking.so
histogram: /usr/local/lib/libpcl_people.so
histogram: /usr/local/lib/libpcl_surface.so
histogram: /usr/local/lib/libpcl_apps.so
histogram: /usr/lib/libvtkViews.so.5.8.0
histogram: /usr/lib/libvtkInfovis.so.5.8.0
histogram: /usr/lib/libvtkWidgets.so.5.8.0
histogram: /usr/lib/libvtkVolumeRendering.so.5.8.0
histogram: /usr/lib/libvtkHybrid.so.5.8.0
histogram: /usr/lib/libvtkParallel.so.5.8.0
histogram: /usr/lib/libvtkRendering.so.5.8.0
histogram: /usr/lib/libvtkImaging.so.5.8.0
histogram: /usr/lib/libvtkGraphics.so.5.8.0
histogram: /usr/lib/libvtkIO.so.5.8.0
histogram: /usr/lib/libvtkFiltering.so.5.8.0
histogram: /usr/lib/libvtkCommon.so.5.8.0
histogram: /usr/lib/libvtksys.so.5.8.0
histogram: CMakeFiles/histogram.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable histogram"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/histogram.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/histogram.dir/build: histogram
.PHONY : CMakeFiles/histogram.dir/build

CMakeFiles/histogram.dir/requires: CMakeFiles/histogram.dir/histogram.cpp.o.requires
CMakeFiles/histogram.dir/requires: CMakeFiles/histogram.dir/planar_filtering.cpp.o.requires
CMakeFiles/histogram.dir/requires: CMakeFiles/histogram.dir/recognition_features.cpp.o.requires
.PHONY : CMakeFiles/histogram.dir/requires

CMakeFiles/histogram.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/histogram.dir/cmake_clean.cmake
.PHONY : CMakeFiles/histogram.dir/clean

CMakeFiles/histogram.dir/depend:
	cd /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build /home/uicmvl/BitBucketRepo/object-recognition/Histogram_Matlab/build/CMakeFiles/histogram.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/histogram.dir/depend

