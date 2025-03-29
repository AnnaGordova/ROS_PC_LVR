# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robot/ros2_ws/src/kiss-icp/ros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robot/ros2_ws/build/kiss_icp

# Include any dependencies generated for this target.
include kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/compiler_depend.make

# Include the progress variables for this target.
include kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/progress.make

# Include the compile flags for this target's objects.
include kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/flags.make

kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o: kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/flags.make
kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o: /home/robot/ros2_ws/src/kiss-icp/cpp/kiss_icp/metrics/Metrics.cpp
kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o: kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robot/ros2_ws/build/kiss_icp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o"
	cd /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o -MF CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o.d -o CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o -c /home/robot/ros2_ws/src/kiss-icp/cpp/kiss_icp/metrics/Metrics.cpp

kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.i"
	cd /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robot/ros2_ws/src/kiss-icp/cpp/kiss_icp/metrics/Metrics.cpp > CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.i

kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.s"
	cd /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robot/ros2_ws/src/kiss-icp/cpp/kiss_icp/metrics/Metrics.cpp -o CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.s

# Object files for target kiss_icp_metrics
kiss_icp_metrics_OBJECTS = \
"CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o"

# External object files for target kiss_icp_metrics
kiss_icp_metrics_EXTERNAL_OBJECTS =

kiss_icp/metrics/libkiss_icp_metrics.a: kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/Metrics.cpp.o
kiss_icp/metrics/libkiss_icp_metrics.a: kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/build.make
kiss_icp/metrics/libkiss_icp_metrics.a: kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robot/ros2_ws/build/kiss_icp/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libkiss_icp_metrics.a"
	cd /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics && $(CMAKE_COMMAND) -P CMakeFiles/kiss_icp_metrics.dir/cmake_clean_target.cmake
	cd /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kiss_icp_metrics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/build: kiss_icp/metrics/libkiss_icp_metrics.a
.PHONY : kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/build

kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/clean:
	cd /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics && $(CMAKE_COMMAND) -P CMakeFiles/kiss_icp_metrics.dir/cmake_clean.cmake
.PHONY : kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/clean

kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/depend:
	cd /home/robot/ros2_ws/build/kiss_icp && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robot/ros2_ws/src/kiss-icp/ros /home/robot/ros2_ws/src/kiss-icp/cpp/kiss_icp/metrics /home/robot/ros2_ws/build/kiss_icp /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics /home/robot/ros2_ws/build/kiss_icp/kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : kiss_icp/metrics/CMakeFiles/kiss_icp_metrics.dir/depend

