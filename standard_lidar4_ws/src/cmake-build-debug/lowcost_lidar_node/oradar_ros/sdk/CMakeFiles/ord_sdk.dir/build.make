# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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
CMAKE_COMMAND = /home/zxj/下载/CLion-2021.1.3/clion-2021.1.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zxj/下载/CLion-2021.1.3/clion-2021.1.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug

# Include any dependencies generated for this target.
include lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/depend.make

# Include the progress variables for this target.
include lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/progress.make

# Include the compile flags for this target's objects.
include lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/flags.make

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.o: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/flags.make
lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.o: ../lowcost_lidar_node/oradar_ros/sdk/src/ord_driver.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.o"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.o -c /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver.cpp

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.i"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver.cpp > CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.i

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.s"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver.cpp -o CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.s

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.o: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/flags.make
lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.o: ../lowcost_lidar_node/oradar_ros/sdk/src/lidar_address.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.o"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.o -c /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/lidar_address.cpp

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.i"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/lidar_address.cpp > CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.i

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.s"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/lidar_address.cpp -o CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.s

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.o: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/flags.make
lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.o: ../lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_impl.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.o"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.o -c /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_impl.cpp

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.i"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_impl.cpp > CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.i

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.s"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_impl.cpp -o CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.s

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.o: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/flags.make
lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.o: ../lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_net.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.o"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.o -c /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_net.cpp

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.i"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_net.cpp > CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.i

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.s"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk/src/ord_driver_net.cpp -o CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.s

# Object files for target ord_sdk
ord_sdk_OBJECTS = \
"CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.o" \
"CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.o" \
"CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.o" \
"CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.o"

# External object files for target ord_sdk
ord_sdk_EXTERNAL_OBJECTS =

devel/lib/ord_sdk.a: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver.cpp.o
devel/lib/ord_sdk.a: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/lidar_address.cpp.o
devel/lib/ord_sdk.a: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_impl.cpp.o
devel/lib/ord_sdk.a: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/src/ord_driver_net.cpp.o
devel/lib/ord_sdk.a: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/build.make
devel/lib/ord_sdk.a: lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX static library ../../../devel/lib/ord_sdk.a"
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && $(CMAKE_COMMAND) -P CMakeFiles/ord_sdk.dir/cmake_clean_target.cmake
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ord_sdk.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/build: devel/lib/ord_sdk.a

.PHONY : lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/build

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/clean:
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk && $(CMAKE_COMMAND) -P CMakeFiles/ord_sdk.dir/cmake_clean.cmake
.PHONY : lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/clean

lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/depend:
	cd /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/lowcost_lidar_node/oradar_ros/sdk /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk /home/zxj/my_code/standard_lidar_driver/standard_lidar4_ws/src/cmake-build-debug/lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lowcost_lidar_node/oradar_ros/sdk/CMakeFiles/ord_sdk.dir/depend

