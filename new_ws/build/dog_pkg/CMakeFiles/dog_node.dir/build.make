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
include dog_pkg/CMakeFiles/dog_node.dir/depend.make

# Include the progress variables for this target.
include dog_pkg/CMakeFiles/dog_node.dir/progress.make

# Include the compile flags for this target's objects.
include dog_pkg/CMakeFiles/dog_node.dir/flags.make

dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o: dog_pkg/CMakeFiles/dog_node.dir/flags.make
dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o: /home/xianzhix/new_ws/src/dog_pkg/src/dog_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/xianzhix/new_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o"
	cd /home/xianzhix/new_ws/build/dog_pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dog_node.dir/src/dog_node.cpp.o -c /home/xianzhix/new_ws/src/dog_pkg/src/dog_node.cpp

dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dog_node.dir/src/dog_node.cpp.i"
	cd /home/xianzhix/new_ws/build/dog_pkg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xianzhix/new_ws/src/dog_pkg/src/dog_node.cpp > CMakeFiles/dog_node.dir/src/dog_node.cpp.i

dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dog_node.dir/src/dog_node.cpp.s"
	cd /home/xianzhix/new_ws/build/dog_pkg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xianzhix/new_ws/src/dog_pkg/src/dog_node.cpp -o CMakeFiles/dog_node.dir/src/dog_node.cpp.s

dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.requires:

.PHONY : dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.requires

dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.provides: dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.requires
	$(MAKE) -f dog_pkg/CMakeFiles/dog_node.dir/build.make dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.provides.build
.PHONY : dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.provides

dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.provides.build: dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o


# Object files for target dog_node
dog_node_OBJECTS = \
"CMakeFiles/dog_node.dir/src/dog_node.cpp.o"

# External object files for target dog_node
dog_node_EXTERNAL_OBJECTS =

/home/xianzhix/new_ws/devel/lib/dog_pkg/dog_node: dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o
/home/xianzhix/new_ws/devel/lib/dog_pkg/dog_node: dog_pkg/CMakeFiles/dog_node.dir/build.make
/home/xianzhix/new_ws/devel/lib/dog_pkg/dog_node: dog_pkg/CMakeFiles/dog_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/xianzhix/new_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/xianzhix/new_ws/devel/lib/dog_pkg/dog_node"
	cd /home/xianzhix/new_ws/build/dog_pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dog_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
dog_pkg/CMakeFiles/dog_node.dir/build: /home/xianzhix/new_ws/devel/lib/dog_pkg/dog_node

.PHONY : dog_pkg/CMakeFiles/dog_node.dir/build

dog_pkg/CMakeFiles/dog_node.dir/requires: dog_pkg/CMakeFiles/dog_node.dir/src/dog_node.cpp.o.requires

.PHONY : dog_pkg/CMakeFiles/dog_node.dir/requires

dog_pkg/CMakeFiles/dog_node.dir/clean:
	cd /home/xianzhix/new_ws/build/dog_pkg && $(CMAKE_COMMAND) -P CMakeFiles/dog_node.dir/cmake_clean.cmake
.PHONY : dog_pkg/CMakeFiles/dog_node.dir/clean

dog_pkg/CMakeFiles/dog_node.dir/depend:
	cd /home/xianzhix/new_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xianzhix/new_ws/src /home/xianzhix/new_ws/src/dog_pkg /home/xianzhix/new_ws/build /home/xianzhix/new_ws/build/dog_pkg /home/xianzhix/new_ws/build/dog_pkg/CMakeFiles/dog_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : dog_pkg/CMakeFiles/dog_node.dir/depend

