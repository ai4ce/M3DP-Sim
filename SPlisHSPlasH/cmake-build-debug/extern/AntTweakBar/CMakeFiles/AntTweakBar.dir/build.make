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
include extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/depend.make

# Include the progress variables for this target.
include extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/progress.make

# Include the compile flags for this target's objects.
include extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.o: ../extern/AntTweakBar/src/LoadOGL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/LoadOGL.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/LoadOGL.cpp > CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/LoadOGL.cpp -o CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.o: ../extern/AntTweakBar/src/LoadOGLCore.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/LoadOGLCore.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/LoadOGLCore.cpp > CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/LoadOGLCore.cpp -o CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.o: ../extern/AntTweakBar/src/TwBar.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwBar.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwBar.cpp > CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwBar.cpp -o CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.o: ../extern/AntTweakBar/src/TwColors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwColors.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwColors.cpp > CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwColors.cpp -o CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.o: ../extern/AntTweakBar/src/TwEventGLFW.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.o   -c /home/jason/splisplash/extern/AntTweakBar/src/TwEventGLFW.c

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwEventGLFW.c > CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwEventGLFW.c -o CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.o: ../extern/AntTweakBar/src/TwEventGLUT.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.o   -c /home/jason/splisplash/extern/AntTweakBar/src/TwEventGLUT.c

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwEventGLUT.c > CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwEventGLUT.c -o CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.o: ../extern/AntTweakBar/src/TwEventSDL.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.o   -c /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL.c

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL.c > CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL.c -o CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.o: ../extern/AntTweakBar/src/TwEventSDL12.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.o   -c /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL12.c

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL12.c > CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL12.c -o CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.o: ../extern/AntTweakBar/src/TwEventSDL13.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.o   -c /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL13.c

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL13.c > CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwEventSDL13.c -o CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.o: ../extern/AntTweakBar/src/TwEventSFML.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwEventSFML.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwEventSFML.cpp > CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwEventSFML.cpp -o CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.o: ../extern/AntTweakBar/src/TwFonts.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwFonts.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwFonts.cpp > CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwFonts.cpp -o CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.o: ../extern/AntTweakBar/src/TwMgr.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwMgr.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwMgr.cpp > CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwMgr.cpp -o CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.o: ../extern/AntTweakBar/src/TwOpenGL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwOpenGL.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwOpenGL.cpp > CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwOpenGL.cpp -o CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.o: ../extern/AntTweakBar/src/TwOpenGLCore.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwOpenGLCore.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwOpenGLCore.cpp > CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwOpenGLCore.cpp -o CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.s

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.o: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/flags.make
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.o: ../extern/AntTweakBar/src/TwPrecomp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.o"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.o -c /home/jason/splisplash/extern/AntTweakBar/src/TwPrecomp.cpp

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.i"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jason/splisplash/extern/AntTweakBar/src/TwPrecomp.cpp > CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.i

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.s"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jason/splisplash/extern/AntTweakBar/src/TwPrecomp.cpp -o CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.s

# Object files for target AntTweakBar
AntTweakBar_OBJECTS = \
"CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.o" \
"CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.o" \
"CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.o" \
"CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.o" \
"CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.o" \
"CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.o" \
"CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.o"

# External object files for target AntTweakBar
AntTweakBar_EXTERNAL_OBJECTS =

lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGL.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/LoadOGLCore.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwBar.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwColors.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLFW.c.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventGLUT.c.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL.c.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL12.c.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSDL13.c.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwEventSFML.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwFonts.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwMgr.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGL.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwOpenGLCore.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/src/TwPrecomp.cpp.o
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/build.make
lib/libAntTweakBar_d.a: extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jason/splisplash/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Linking CXX static library ../../lib/libAntTweakBar_d.a"
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && $(CMAKE_COMMAND) -P CMakeFiles/AntTweakBar.dir/cmake_clean_target.cmake
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AntTweakBar.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/build: lib/libAntTweakBar_d.a

.PHONY : extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/build

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/clean:
	cd /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar && $(CMAKE_COMMAND) -P CMakeFiles/AntTweakBar.dir/cmake_clean.cmake
.PHONY : extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/clean

extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/depend:
	cd /home/jason/splisplash/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jason/splisplash /home/jason/splisplash/extern/AntTweakBar /home/jason/splisplash/cmake-build-debug /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar /home/jason/splisplash/cmake-build-debug/extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : extern/AntTweakBar/CMakeFiles/AntTweakBar.dir/depend
