# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/dmrud/Git/pcl_tutor/visualisation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dmrud/Git/pcl_tutor/visualisation/build

# Include any dependencies generated for this target.
include CMakeFiles/cloud_visualizer_rgba.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cloud_visualizer_rgba.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cloud_visualizer_rgba.dir/flags.make

CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.o: CMakeFiles/cloud_visualizer_rgba.dir/flags.make
CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.o: ../visualize_pcd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmrud/Git/pcl_tutor/visualisation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.o -c /home/dmrud/Git/pcl_tutor/visualisation/visualize_pcd.cpp

CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmrud/Git/pcl_tutor/visualisation/visualize_pcd.cpp > CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.i

CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmrud/Git/pcl_tutor/visualisation/visualize_pcd.cpp -o CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.s

# Object files for target cloud_visualizer_rgba
cloud_visualizer_rgba_OBJECTS = \
"CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.o"

# External object files for target cloud_visualizer_rgba
cloud_visualizer_rgba_EXTERNAL_OBJECTS =

cloud_visualizer_rgba: CMakeFiles/cloud_visualizer_rgba.dir/visualize_pcd.cpp.o
cloud_visualizer_rgba: CMakeFiles/cloud_visualizer_rgba.dir/build.make
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_people.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libboost_system.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libboost_regex.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libqhull.so
cloud_visualizer_rgba: /usr/lib/libOpenNI.so
cloud_visualizer_rgba: /usr/lib/libOpenNI2.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libfreetype.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libz.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libjpeg.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpng.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libtiff.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libexpat.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_features.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_search.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_io.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libpcl_common.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libfreetype.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libz.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libGLEW.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libSM.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libICE.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libX11.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libXext.so
cloud_visualizer_rgba: /usr/lib/x86_64-linux-gnu/libXt.so
cloud_visualizer_rgba: CMakeFiles/cloud_visualizer_rgba.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dmrud/Git/pcl_tutor/visualisation/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable cloud_visualizer_rgba"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cloud_visualizer_rgba.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cloud_visualizer_rgba.dir/build: cloud_visualizer_rgba

.PHONY : CMakeFiles/cloud_visualizer_rgba.dir/build

CMakeFiles/cloud_visualizer_rgba.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cloud_visualizer_rgba.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cloud_visualizer_rgba.dir/clean

CMakeFiles/cloud_visualizer_rgba.dir/depend:
	cd /home/dmrud/Git/pcl_tutor/visualisation/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dmrud/Git/pcl_tutor/visualisation /home/dmrud/Git/pcl_tutor/visualisation /home/dmrud/Git/pcl_tutor/visualisation/build /home/dmrud/Git/pcl_tutor/visualisation/build /home/dmrud/Git/pcl_tutor/visualisation/build/CMakeFiles/cloud_visualizer_rgba.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cloud_visualizer_rgba.dir/depend

