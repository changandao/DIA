# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/changandao/Documents/clion-2018.3.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/changandao/Documents/clion-2018.3.1/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/changandao/SLAM/dia/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/changandao/SLAM/dia/src/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/myslam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myslam.dir/flags.make

CMakeFiles/myslam.dir/frame.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/frame.cpp.o: ../frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myslam.dir/frame.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/frame.cpp.o -c /home/changandao/SLAM/dia/src/frame.cpp

CMakeFiles/myslam.dir/frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/frame.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changandao/SLAM/dia/src/frame.cpp > CMakeFiles/myslam.dir/frame.cpp.i

CMakeFiles/myslam.dir/frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/frame.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changandao/SLAM/dia/src/frame.cpp -o CMakeFiles/myslam.dir/frame.cpp.s

CMakeFiles/myslam.dir/pyrframes.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/pyrframes.cpp.o: ../pyrframes.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/myslam.dir/pyrframes.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/pyrframes.cpp.o -c /home/changandao/SLAM/dia/src/pyrframes.cpp

CMakeFiles/myslam.dir/pyrframes.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/pyrframes.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changandao/SLAM/dia/src/pyrframes.cpp > CMakeFiles/myslam.dir/pyrframes.cpp.i

CMakeFiles/myslam.dir/pyrframes.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/pyrframes.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changandao/SLAM/dia/src/pyrframes.cpp -o CMakeFiles/myslam.dir/pyrframes.cpp.s

CMakeFiles/myslam.dir/camera.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/camera.cpp.o: ../camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/myslam.dir/camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/camera.cpp.o -c /home/changandao/SLAM/dia/src/camera.cpp

CMakeFiles/myslam.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changandao/SLAM/dia/src/camera.cpp > CMakeFiles/myslam.dir/camera.cpp.i

CMakeFiles/myslam.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changandao/SLAM/dia/src/camera.cpp -o CMakeFiles/myslam.dir/camera.cpp.s

CMakeFiles/myslam.dir/config.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/config.cpp.o: ../config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/myslam.dir/config.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/config.cpp.o -c /home/changandao/SLAM/dia/src/config.cpp

CMakeFiles/myslam.dir/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/config.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changandao/SLAM/dia/src/config.cpp > CMakeFiles/myslam.dir/config.cpp.i

CMakeFiles/myslam.dir/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/config.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changandao/SLAM/dia/src/config.cpp -o CMakeFiles/myslam.dir/config.cpp.s

CMakeFiles/myslam.dir/g2o_types.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/g2o_types.cpp.o: ../g2o_types.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/myslam.dir/g2o_types.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/g2o_types.cpp.o -c /home/changandao/SLAM/dia/src/g2o_types.cpp

CMakeFiles/myslam.dir/g2o_types.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/g2o_types.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changandao/SLAM/dia/src/g2o_types.cpp > CMakeFiles/myslam.dir/g2o_types.cpp.i

CMakeFiles/myslam.dir/g2o_types.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/g2o_types.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changandao/SLAM/dia/src/g2o_types.cpp -o CMakeFiles/myslam.dir/g2o_types.cpp.s

CMakeFiles/myslam.dir/visual_odometry.cpp.o: CMakeFiles/myslam.dir/flags.make
CMakeFiles/myslam.dir/visual_odometry.cpp.o: ../visual_odometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/myslam.dir/visual_odometry.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/visual_odometry.cpp.o -c /home/changandao/SLAM/dia/src/visual_odometry.cpp

CMakeFiles/myslam.dir/visual_odometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/visual_odometry.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/changandao/SLAM/dia/src/visual_odometry.cpp > CMakeFiles/myslam.dir/visual_odometry.cpp.i

CMakeFiles/myslam.dir/visual_odometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/visual_odometry.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/changandao/SLAM/dia/src/visual_odometry.cpp -o CMakeFiles/myslam.dir/visual_odometry.cpp.s

# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/frame.cpp.o" \
"CMakeFiles/myslam.dir/pyrframes.cpp.o" \
"CMakeFiles/myslam.dir/camera.cpp.o" \
"CMakeFiles/myslam.dir/config.cpp.o" \
"CMakeFiles/myslam.dir/g2o_types.cpp.o" \
"CMakeFiles/myslam.dir/visual_odometry.cpp.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

libmyslam.so: CMakeFiles/myslam.dir/frame.cpp.o
libmyslam.so: CMakeFiles/myslam.dir/pyrframes.cpp.o
libmyslam.so: CMakeFiles/myslam.dir/camera.cpp.o
libmyslam.so: CMakeFiles/myslam.dir/config.cpp.o
libmyslam.so: CMakeFiles/myslam.dir/g2o_types.cpp.o
libmyslam.so: CMakeFiles/myslam.dir/visual_odometry.cpp.o
libmyslam.so: CMakeFiles/myslam.dir/build.make
libmyslam.so: CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX shared library libmyslam.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myslam.dir/build: libmyslam.so

.PHONY : CMakeFiles/myslam.dir/build

CMakeFiles/myslam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myslam.dir/clean

CMakeFiles/myslam.dir/depend:
	cd /home/changandao/SLAM/dia/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/changandao/SLAM/dia/src /home/changandao/SLAM/dia/src /home/changandao/SLAM/dia/src/cmake-build-debug /home/changandao/SLAM/dia/src/cmake-build-debug /home/changandao/SLAM/dia/src/cmake-build-debug/CMakeFiles/myslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myslam.dir/depend

