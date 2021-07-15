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
include Tests/Kernel/CMakeFiles/KernelTests.dir/depend.make

# Include the progress variables for this target.
include Tests/Kernel/CMakeFiles/KernelTests.dir/progress.make

# Include the compile flags for this target's objects.
include Tests/Kernel/CMakeFiles/KernelTests.dir/flags.make

Tests/Kernel/CMakeFiles/KernelTests.dir/KernelTests.cpp.o: Tests/Kernel/CMakeFiles/KernelTests.dir/flags.make
Tests/Kernel/CMakeFiles/KernelTests.dir/KernelTests.cpp.o: ../Tests/Kernel/KernelTests.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Tests/Kernel/CMakeFiles/KernelTests.dir/KernelTests.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/Tests/Kernel && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/KernelTests.dir/KernelTests.cpp.o -c /home/jason/splisplash/Tests/Kernel/KernelTests.cpp

Tests/Kernel/CMakeFiles/KernelTests.dir/KernelTests.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/KernelTests.dir/KernelTests.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/Tests/Kernel && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/Tests/Kernel/KernelTests.cpp > CMakeFiles/KernelTests.dir/KernelTests.cpp.i

Tests/Kernel/CMakeFiles/KernelTests.dir/KernelTests.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/KernelTests.dir/KernelTests.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/Tests/Kernel && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/Tests/Kernel/KernelTests.cpp -o CMakeFiles/KernelTests.dir/KernelTests.cpp.s

# Object files for target KernelTests
KernelTests_OBJECTS = \
"CMakeFiles/KernelTests.dir/KernelTests.cpp.o"

# External object files for target KernelTests
KernelTests_EXTERNAL_OBJECTS =

../bin/KernelTests_d: Tests/Kernel/CMakeFiles/KernelTests.dir/KernelTests.cpp.o
../bin/KernelTests_d: Tests/Kernel/CMakeFiles/KernelTests.dir/build.make
../bin/KernelTests_d: lib/libSPlisHSPlasH_d.a
../bin/KernelTests_d: lib/libUtilities_d.a
../bin/KernelTests_d: lib/libpartio_d.a
../bin/KernelTests_d: lib/libzlib_d.a
../bin/KernelTests_d: lib/libMD5_d.a
../bin/KernelTests_d: lib/libtinyexpr_d.a
../bin/KernelTests_d: extern/install/NeighborhoodSearch/lib/libCompactNSearch_d.a
../bin/KernelTests_d: extern/install/Discregrid/lib/libDiscregrid_d.a
../bin/KernelTests_d: Tests/Kernel/CMakeFiles/KernelTests.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../../bin/KernelTests_d"
	cd /home/jason/splisplash/cmake-build-debug/Tests/Kernel && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/KernelTests.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Tests/Kernel/CMakeFiles/KernelTests.dir/build: ../bin/KernelTests_d

.PHONY : Tests/Kernel/CMakeFiles/KernelTests.dir/build

Tests/Kernel/CMakeFiles/KernelTests.dir/clean:
	cd /home/jason/splisplash/cmake-build-debug/Tests/Kernel && $(CMAKE_COMMAND) -P CMakeFiles/KernelTests.dir/cmake_clean.cmake
.PHONY : Tests/Kernel/CMakeFiles/KernelTests.dir/clean

Tests/Kernel/CMakeFiles/KernelTests.dir/depend:
	cd /home/jason/splisplash/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jason/splisplash /home/jason/splisplash/Tests/Kernel /home/jason/splisplash/cmake-build-debug /home/jason/splisplash/cmake-build-debug/Tests/Kernel /home/jason/splisplash/cmake-build-debug/Tests/Kernel/CMakeFiles/KernelTests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Tests/Kernel/CMakeFiles/KernelTests.dir/depend
