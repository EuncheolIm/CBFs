# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.27

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.27.1/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.27.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/im-euncheol/Desktop/KIST/CBFs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/im-euncheol/Desktop/KIST/CBFs/build

# Include any dependencies generated for this target.
include libs/mujoco/CMakeFiles/lodepng.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include libs/mujoco/CMakeFiles/lodepng.dir/compiler_depend.make

# Include the progress variables for this target.
include libs/mujoco/CMakeFiles/lodepng.dir/progress.make

# Include the compile flags for this target's objects.
include libs/mujoco/CMakeFiles/lodepng.dir/flags.make

libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o: libs/mujoco/CMakeFiles/lodepng.dir/flags.make
libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o: _deps/lodepng-src/lodepng.cpp
libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o: libs/mujoco/CMakeFiles/lodepng.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o -MF CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o.d -o CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o -c /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/lodepng-src/lodepng.cpp

libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.i"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/lodepng-src/lodepng.cpp > CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.i

libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.s"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco && /usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/lodepng-src/lodepng.cpp -o CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.s

# Object files for target lodepng
lodepng_OBJECTS = \
"CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o"

# External object files for target lodepng
lodepng_EXTERNAL_OBJECTS =

lib/liblodepng.a: libs/mujoco/CMakeFiles/lodepng.dir/__/__/_deps/lodepng-src/lodepng.cpp.o
lib/liblodepng.a: libs/mujoco/CMakeFiles/lodepng.dir/build.make
lib/liblodepng.a: libs/mujoco/CMakeFiles/lodepng.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library ../../lib/liblodepng.a"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco && $(CMAKE_COMMAND) -P CMakeFiles/lodepng.dir/cmake_clean_target.cmake
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lodepng.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
libs/mujoco/CMakeFiles/lodepng.dir/build: lib/liblodepng.a
.PHONY : libs/mujoco/CMakeFiles/lodepng.dir/build

libs/mujoco/CMakeFiles/lodepng.dir/clean:
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco && $(CMAKE_COMMAND) -P CMakeFiles/lodepng.dir/cmake_clean.cmake
.PHONY : libs/mujoco/CMakeFiles/lodepng.dir/clean

libs/mujoco/CMakeFiles/lodepng.dir/depend:
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/im-euncheol/Desktop/KIST/CBFs /Users/im-euncheol/Desktop/KIST/CBFs/libs/mujoco /Users/im-euncheol/Desktop/KIST/CBFs/build /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco /Users/im-euncheol/Desktop/KIST/CBFs/build/libs/mujoco/CMakeFiles/lodepng.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : libs/mujoco/CMakeFiles/lodepng.dir/depend
