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
CMAKE_SOURCE_DIR = /home/j/EECS598_MotionPlanning/hw03/my_solution

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/j/EECS598_MotionPlanning/hw03/my_solution/build

# Include any dependencies generated for this target.
include CMakeFiles/myplugin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/myplugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/myplugin.dir/flags.make

CMakeFiles/myplugin.dir/myplugin.cpp.o: CMakeFiles/myplugin.dir/flags.make
CMakeFiles/myplugin.dir/myplugin.cpp.o: ../myplugin.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/j/EECS598_MotionPlanning/hw03/my_solution/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/myplugin.dir/myplugin.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myplugin.dir/myplugin.cpp.o -c /home/j/EECS598_MotionPlanning/hw03/my_solution/myplugin.cpp

CMakeFiles/myplugin.dir/myplugin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myplugin.dir/myplugin.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/j/EECS598_MotionPlanning/hw03/my_solution/myplugin.cpp > CMakeFiles/myplugin.dir/myplugin.cpp.i

CMakeFiles/myplugin.dir/myplugin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myplugin.dir/myplugin.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/j/EECS598_MotionPlanning/hw03/my_solution/myplugin.cpp -o CMakeFiles/myplugin.dir/myplugin.cpp.s

CMakeFiles/myplugin.dir/myplugin.cpp.o.requires:

.PHONY : CMakeFiles/myplugin.dir/myplugin.cpp.o.requires

CMakeFiles/myplugin.dir/myplugin.cpp.o.provides: CMakeFiles/myplugin.dir/myplugin.cpp.o.requires
	$(MAKE) -f CMakeFiles/myplugin.dir/build.make CMakeFiles/myplugin.dir/myplugin.cpp.o.provides.build
.PHONY : CMakeFiles/myplugin.dir/myplugin.cpp.o.provides

CMakeFiles/myplugin.dir/myplugin.cpp.o.provides.build: CMakeFiles/myplugin.dir/myplugin.cpp.o


# Object files for target myplugin
myplugin_OBJECTS = \
"CMakeFiles/myplugin.dir/myplugin.cpp.o"

# External object files for target myplugin
myplugin_EXTERNAL_OBJECTS =

libmyplugin.so: CMakeFiles/myplugin.dir/myplugin.cpp.o
libmyplugin.so: CMakeFiles/myplugin.dir/build.make
libmyplugin.so: Head/libStructSelfDefine.so
libmyplugin.so: alglib-3.16.0.cpp.gpl/cpp/src/libalglib.so
libmyplugin.so: CMakeFiles/myplugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/j/EECS598_MotionPlanning/hw03/my_solution/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libmyplugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myplugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/myplugin.dir/build: libmyplugin.so

.PHONY : CMakeFiles/myplugin.dir/build

CMakeFiles/myplugin.dir/requires: CMakeFiles/myplugin.dir/myplugin.cpp.o.requires

.PHONY : CMakeFiles/myplugin.dir/requires

CMakeFiles/myplugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/myplugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/myplugin.dir/clean

CMakeFiles/myplugin.dir/depend:
	cd /home/j/EECS598_MotionPlanning/hw03/my_solution/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/j/EECS598_MotionPlanning/hw03/my_solution /home/j/EECS598_MotionPlanning/hw03/my_solution /home/j/EECS598_MotionPlanning/hw03/my_solution/build /home/j/EECS598_MotionPlanning/hw03/my_solution/build /home/j/EECS598_MotionPlanning/hw03/my_solution/build/CMakeFiles/myplugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/myplugin.dir/depend

