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
include Utilities/CMakeFiles/Utilities.dir/depend.make

# Include the progress variables for this target.
include Utilities/CMakeFiles/Utilities.dir/progress.make

# Include the compile flags for this target's objects.
include Utilities/CMakeFiles/Utilities.dir/flags.make

Utilities/CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.o: Utilities/CMakeFiles/Utilities.dir/flags.make
Utilities/CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.o: ../Utilities/PartioReaderWriter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Utilities/CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/Utilities && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.o -c /home/jason/splisplash/Utilities/PartioReaderWriter.cpp

Utilities/CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/Utilities && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/Utilities/PartioReaderWriter.cpp > CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.i

Utilities/CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/Utilities && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/Utilities/PartioReaderWriter.cpp -o CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.s

# Object files for target Utilities
Utilities_OBJECTS = \
"CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.o"

# External object files for target Utilities
Utilities_EXTERNAL_OBJECTS =

lib/libUtilities_d.a: Utilities/CMakeFiles/Utilities.dir/PartioReaderWriter.cpp.o
lib/libUtilities_d.a: Utilities/CMakeFiles/Utilities.dir/build.make
lib/libUtilities_d.a: Utilities/CMakeFiles/Utilities.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../lib/libUtilities_d.a"
	cd /home/jason/splisplash/cmake-build-debug/Utilities && $(CMAKE_COMMAND) -P CMakeFiles/Utilities.dir/cmake_clean_target.cmake
	cd /home/jason/splisplash/cmake-build-debug/Utilities && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Utilities.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Utilities/CMakeFiles/Utilities.dir/build: lib/libUtilities_d.a

.PHONY : Utilities/CMakeFiles/Utilities.dir/build

Utilities/CMakeFiles/Utilities.dir/clean:
	cd /home/jason/splisplash/cmake-build-debug/Utilities && $(CMAKE_COMMAND) -P CMakeFiles/Utilities.dir/cmake_clean.cmake
.PHONY : Utilities/CMakeFiles/Utilities.dir/clean

Utilities/CMakeFiles/Utilities.dir/depend:
	cd /home/jason/splisplash/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jason/splisplash /home/jason/splisplash/Utilities /home/jason/splisplash/cmake-build-debug /home/jason/splisplash/cmake-build-debug/Utilities /home/jason/splisplash/cmake-build-debug/Utilities/CMakeFiles/Utilities.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Utilities/CMakeFiles/Utilities.dir/depend

