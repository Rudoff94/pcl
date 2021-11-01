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
CMAKE_SOURCE_DIR = /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build

# Include any dependencies generated for this target.
include CMakeFiles/statistical_removal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/statistical_removal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/statistical_removal.dir/flags.make

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o: CMakeFiles/statistical_removal.dir/flags.make
CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o: ../statistical_removal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o -c /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/statistical_removal.cpp

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/statistical_removal.cpp > CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/statistical_removal.cpp -o CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s

# Object files for target statistical_removal
statistical_removal_OBJECTS = \
"CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o"

# External object files for target statistical_removal
statistical_removal_EXTERNAL_OBJECTS =

statistical_removal: CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o
statistical_removal: CMakeFiles/statistical_removal.dir/build.make
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_apps.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_outofcore.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_people.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_system.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_regex.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libqhull.so
statistical_removal: /usr/lib/libOpenNI.so
statistical_removal: /usr/lib/libOpenNI2.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libfreetype.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libz.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libjpeg.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpng.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libtiff.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libexpat.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_surface.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_keypoints.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_tracking.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_recognition.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_registration.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_stereo.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_segmentation.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_features.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_filters.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_sample_consensus.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_ml.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_search.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_kdtree.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_io.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_octree.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpcl_common.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkalglib-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libfreetype.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtksys-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1
statistical_removal: /usr/lib/x86_64-linux-gnu/libz.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libGLEW.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libSM.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libICE.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libX11.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libXext.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libXt.so
statistical_removal: CMakeFiles/statistical_removal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable statistical_removal"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/statistical_removal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/statistical_removal.dir/build: statistical_removal

.PHONY : CMakeFiles/statistical_removal.dir/build

CMakeFiles/statistical_removal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/statistical_removal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/statistical_removal.dir/clean

CMakeFiles/statistical_removal.dir/depend:
	cd /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build /home/dmrud/Git/pcl_tutor/Filtering/statistical_outlier_removal_filter/build/CMakeFiles/statistical_removal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/statistical_removal.dir/depend

