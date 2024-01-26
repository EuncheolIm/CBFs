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

# Utility rule file for update_mappings.

# Include any custom commands dependencies for this target.
include _deps/glfw3-build/src/CMakeFiles/update_mappings.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/glfw3-build/src/CMakeFiles/update_mappings.dir/progress.make

_deps/glfw3-build/src/CMakeFiles/update_mappings:
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Updating gamepad mappings from upstream repository"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/glfw3-src/src && /opt/homebrew/Cellar/cmake/3.27.1/bin/cmake -P /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/glfw3-src/CMake/GenerateMappings.cmake mappings.h.in mappings.h

update_mappings: _deps/glfw3-build/src/CMakeFiles/update_mappings
update_mappings: _deps/glfw3-build/src/CMakeFiles/update_mappings.dir/build.make
.PHONY : update_mappings

# Rule to build all files generated by this target.
_deps/glfw3-build/src/CMakeFiles/update_mappings.dir/build: update_mappings
.PHONY : _deps/glfw3-build/src/CMakeFiles/update_mappings.dir/build

_deps/glfw3-build/src/CMakeFiles/update_mappings.dir/clean:
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/glfw3-build/src && $(CMAKE_COMMAND) -P CMakeFiles/update_mappings.dir/cmake_clean.cmake
.PHONY : _deps/glfw3-build/src/CMakeFiles/update_mappings.dir/clean

_deps/glfw3-build/src/CMakeFiles/update_mappings.dir/depend:
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/im-euncheol/Desktop/KIST/CBFs /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/glfw3-src/src /Users/im-euncheol/Desktop/KIST/CBFs/build /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/glfw3-build/src /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/glfw3-build/src/CMakeFiles/update_mappings.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/glfw3-build/src/CMakeFiles/update_mappings.dir/depend
