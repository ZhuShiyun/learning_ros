# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/xianzhix/new_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xianzhix/new_ws/build

# Include any dependencies generated for this target.
include learning_service/CMakeFiles/person_client.dir/depend.make

# Include the progress variables for this target.
include learning_service/CMakeFiles/person_client.dir/progress.make

# Include the compile flags for this target's objects.
include learning_service/CMakeFiles/person_client.dir/flags.make

learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o: learning_service/CMakeFiles/person_client.dir/flags.make
learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o: /home/xianzhix/new_ws/src/learning_service/src/person_client.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xianzhix/new_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o"
	cd /home/xianzhix/new_ws/build/learning_service && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/person_client.dir/src/person_client.cpp.o -c /home/xianzhix/new_ws/src/learning_service/src/person_client.cpp

learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/person_client.dir/src/person_client.cpp.i"
	cd /home/xianzhix/new_ws/build/learning_service && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xianzhix/new_ws/src/learning_service/src/person_client.cpp > CMakeFiles/person_client.dir/src/person_client.cpp.i

learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/person_client.dir/src/person_client.cpp.s"
	cd /home/xianzhix/new_ws/build/learning_service && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xianzhix/new_ws/src/learning_service/src/person_client.cpp -o CMakeFiles/person_client.dir/src/person_client.cpp.s

learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.requires:

.PHONY : learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.requires

learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.provides: learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.requires
	$(MAKE) -f learning_service/CMakeFiles/person_client.dir/build.make learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.provides.build
.PHONY : learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.provides

learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.provides.build: learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o


# Object files for target person_client
person_client_OBJECTS = \
"CMakeFiles/person_client.dir/src/person_client.cpp.o"

# External object files for target person_client
person_client_EXTERNAL_OBJECTS =

/home/xianzhix/new_ws/devel/lib/learning_service/person_client: learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: learning_service/CMakeFiles/person_client.dir/build.make
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/libroscpp.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/librosconsole.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/librostime.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /opt/ros/melodic/lib/libcpp_common.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/xianzhix/new_ws/devel/lib/learning_service/person_client: learning_service/CMakeFiles/person_client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xianzhix/new_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/xianzhix/new_ws/devel/lib/learning_service/person_client"
	cd /home/xianzhix/new_ws/build/learning_service && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/person_client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
learning_service/CMakeFiles/person_client.dir/build: /home/xianzhix/new_ws/devel/lib/learning_service/person_client

.PHONY : learning_service/CMakeFiles/person_client.dir/build

learning_service/CMakeFiles/person_client.dir/requires: learning_service/CMakeFiles/person_client.dir/src/person_client.cpp.o.requires

.PHONY : learning_service/CMakeFiles/person_client.dir/requires

learning_service/CMakeFiles/person_client.dir/clean:
	cd /home/xianzhix/new_ws/build/learning_service && $(CMAKE_COMMAND) -P CMakeFiles/person_client.dir/cmake_clean.cmake
.PHONY : learning_service/CMakeFiles/person_client.dir/clean

learning_service/CMakeFiles/person_client.dir/depend:
	cd /home/xianzhix/new_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xianzhix/new_ws/src /home/xianzhix/new_ws/src/learning_service /home/xianzhix/new_ws/build /home/xianzhix/new_ws/build/learning_service /home/xianzhix/new_ws/build/learning_service/CMakeFiles/person_client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : learning_service/CMakeFiles/person_client.dir/depend

