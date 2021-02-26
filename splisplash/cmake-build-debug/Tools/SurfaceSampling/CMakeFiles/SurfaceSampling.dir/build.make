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
CMAKE_COMMAND = /opt/clion-2020.2.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2020.2.4/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jason/splisplash

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jason/splisplash/cmake-build-debug

# Include any dependencies generated for this target.
include Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/depend.make

# Include the progress variables for this target.
include Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/progress.make

# Include the compile flags for this target's objects.
include Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/flags.make

Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/main.cpp.o: Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/flags.make
Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/main.cpp.o: ../Tools/SurfaceSampling/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/main.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SurfaceSampling.dir/main.cpp.o -c /home/jason/splisplash/Tools/SurfaceSampling/main.cpp

Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SurfaceSampling.dir/main.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/Tools/SurfaceSampling/main.cpp > CMakeFiles/SurfaceSampling.dir/main.cpp.i

Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SurfaceSampling.dir/main.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/Tools/SurfaceSampling/main.cpp -o CMakeFiles/SurfaceSampling.dir/main.cpp.s

# Object files for target SurfaceSampling
SurfaceSampling_OBJECTS = \
"CMakeFiles/SurfaceSampling.dir/main.cpp.o"

# External object files for target SurfaceSampling
SurfaceSampling_EXTERNAL_OBJECTS =

../bin/SurfaceSampling_d: Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/main.cpp.o
../bin/SurfaceSampling_d: Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/build.make
../bin/SurfaceSampling_d: lib/libSPlisHSPlasH_d.a
../bin/SurfaceSampling_d: lib/libUtilities_d.a
../bin/SurfaceSampling_d: lib/libpartio_d.a
../bin/SurfaceSampling_d: lib/libzlib_d.a
../bin/SurfaceSampling_d: lib/libMD5_d.a
../bin/SurfaceSampling_d: lib/libtinyexpr_d.a
../bin/SurfaceSampling_d: extern/install/NeighborhoodSearch/lib/libCompactNSearch_d.a
../bin/SurfaceSampling_d: extern/install/Discregrid/lib/libDiscregrid_d.a
../bin/SurfaceSampling_d: Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/SurfaceSampling_d"
	cd /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SurfaceSampling.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/build: ../bin/SurfaceSampling_d

.PHONY : Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/build

Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/clean:
	cd /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling && $(CMAKE_COMMAND) -P CMakeFiles/SurfaceSampling.dir/cmake_clean.cmake
.PHONY : Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/clean

Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/depend:
	cd /home/jason/splisplash/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jason/splisplash /home/jason/splisplash/Tools/SurfaceSampling /home/jason/splisplash/cmake-build-debug /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling /home/jason/splisplash/cmake-build-debug/Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Tools/SurfaceSampling/CMakeFiles/SurfaceSampling.dir/depend

