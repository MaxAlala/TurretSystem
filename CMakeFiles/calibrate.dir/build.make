# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/sol/ides/clion-2020.3.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/sol/ides/clion-2020.3.1/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sol/git_ws/TurretSystem

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sol/git_ws/TurretSystem

# Include any dependencies generated for this target.
include CMakeFiles/calibrate.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/calibrate.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/calibrate.dir/flags.make

CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.o: CMakeFiles/calibrate.dir/flags.make
CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.o: src/calib_intrinsic.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sol/git_ws/TurretSystem/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.o -c /home/sol/git_ws/TurretSystem/src/calib_intrinsic.cpp

CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sol/git_ws/TurretSystem/src/calib_intrinsic.cpp > CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.i

CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sol/git_ws/TurretSystem/src/calib_intrinsic.cpp -o CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.s

# Object files for target calibrate
calibrate_OBJECTS = \
"CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.o"

# External object files for target calibrate
calibrate_EXTERNAL_OBJECTS =

bin/calibrate: CMakeFiles/calibrate.dir/src/calib_intrinsic.cpp.o
bin/calibrate: CMakeFiles/calibrate.dir/build.make
bin/calibrate: CMakeFiles/calibrate.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sol/git_ws/TurretSystem/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/calibrate"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/calibrate.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/calibrate.dir/build: bin/calibrate

.PHONY : CMakeFiles/calibrate.dir/build

CMakeFiles/calibrate.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/calibrate.dir/cmake_clean.cmake
.PHONY : CMakeFiles/calibrate.dir/clean

CMakeFiles/calibrate.dir/depend:
	cd /home/sol/git_ws/TurretSystem && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sol/git_ws/TurretSystem /home/sol/git_ws/TurretSystem /home/sol/git_ws/TurretSystem /home/sol/git_ws/TurretSystem /home/sol/git_ws/TurretSystem/CMakeFiles/calibrate.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/calibrate.dir/depend
