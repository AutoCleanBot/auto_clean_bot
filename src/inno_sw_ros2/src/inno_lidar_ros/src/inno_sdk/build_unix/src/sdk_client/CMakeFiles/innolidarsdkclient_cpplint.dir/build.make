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
CMAKE_SOURCE_DIR = /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix

# Utility rule file for innolidarsdkclient_cpplint.

# Include the progress variables for this target.
include src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/progress.make

src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint: ../src/sdk_client/\ 
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Linting innolidarsdkclient_cpplint"
	cd /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix/src/sdk_client && /usr/bin/cmake -E chdir /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/src/sdk_client /usr/bin/python /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/src/sdk_client/../../build/cpplint.py --filter=-runtime/references,-build/include_subdir,-build/c++11,-build/header_guard,+build/include_what_you_use --counting=detailed --extensions=hpp,cpp,cc,h --linelength=120 --exclude=src/getopt.h " "

innolidarsdkclient_cpplint: src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint
innolidarsdkclient_cpplint: src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/build.make

.PHONY : innolidarsdkclient_cpplint

# Rule to build all files generated by this target.
src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/build: innolidarsdkclient_cpplint

.PHONY : src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/build

src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/clean:
	cd /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix/src/sdk_client && $(CMAKE_COMMAND) -P CMakeFiles/innolidarsdkclient_cpplint.dir/cmake_clean.cmake
.PHONY : src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/clean

src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/depend:
	cd /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/src/sdk_client /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix/src/sdk_client /home/nvidia/auto_clean_bot/src/inno_sw_ros2/src/inno_lidar_ros/src/inno_sdk/build_unix/src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/sdk_client/CMakeFiles/innolidarsdkclient_cpplint.dir/depend

