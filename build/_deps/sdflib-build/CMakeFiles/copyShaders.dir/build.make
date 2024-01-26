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

# Utility rule file for copyShaders.

# Include any custom commands dependencies for this target.
include _deps/sdflib-build/CMakeFiles/copyShaders.dir/compiler_depend.make

# Include the progress variables for this target.
include _deps/sdflib-build/CMakeFiles/copyShaders.dir/progress.make

_deps/sdflib-build/basic.frag: _deps/sdflib-src/src/render_engine/shaders/basic.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating basic.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/basic.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/basic.frag

_deps/sdflib-build/basic.vert: _deps/sdflib-src/src/render_engine/shaders/basic.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating basic.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/basic.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/basic.vert

_deps/sdflib-build/colors.frag: _deps/sdflib-src/src/render_engine/shaders/colors.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating colors.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/colors.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/colors.frag

_deps/sdflib-build/colors.vert: _deps/sdflib-src/src/render_engine/shaders/colors.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating colors.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/colors.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/colors.vert

_deps/sdflib-build/grid.frag: _deps/sdflib-src/src/render_engine/shaders/grid.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating grid.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/grid.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/grid.frag

_deps/sdflib-build/grid.vert: _deps/sdflib-src/src/render_engine/shaders/grid.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating grid.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/grid.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/grid.vert

_deps/sdflib-build/normals.frag: _deps/sdflib-src/src/render_engine/shaders/normals.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating normals.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/normals.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/normals.frag

_deps/sdflib-build/normals.vert: _deps/sdflib-src/src/render_engine/shaders/normals.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating normals.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/normals.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/normals.vert

_deps/sdflib-build/normalsSplitPlane.frag: _deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating normalsSplitPlane.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/normalsSplitPlane.frag

_deps/sdflib-build/normalsSplitPlane.vert: _deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Generating normalsSplitPlane.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/normalsSplitPlane.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/normalsSplitPlane.vert

_deps/sdflib-build/screenPlane.frag: _deps/sdflib-src/src/render_engine/shaders/screenPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Generating screenPlane.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/screenPlane.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/screenPlane.frag

_deps/sdflib-build/screenPlane.vert: _deps/sdflib-src/src/render_engine/shaders/screenPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Generating screenPlane.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/screenPlane.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/screenPlane.vert

_deps/sdflib-build/sdfOctreeLight.frag: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Generating sdfOctreeLight.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreeLight.frag

_deps/sdflib-build/sdfOctreeLight.vert: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Generating sdfOctreeLight.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeLight.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreeLight.vert

_deps/sdflib-build/sdfOctreeMeanTrianglesPlane.frag: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Generating sdfOctreeMeanTrianglesPlane.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreeMeanTrianglesPlane.frag

_deps/sdflib-build/sdfOctreeMeanTrianglesPlane.vert: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Generating sdfOctreeMeanTrianglesPlane.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeMeanTrianglesPlane.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreeMeanTrianglesPlane.vert

_deps/sdflib-build/sdfOctreePlane.frag: _deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Generating sdfOctreePlane.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreePlane.frag

_deps/sdflib-build/sdfOctreePlane.vert: _deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Generating sdfOctreePlane.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreePlane.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreePlane.vert

_deps/sdflib-build/sdfOctreeRender.comp: _deps/sdflib-src/src/render_engine/shaders/sdfOctreeRender.comp
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Generating sdfOctreeRender.comp"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfOctreeRender.comp /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfOctreeRender.comp

_deps/sdflib-build/sdfPlane.frag: _deps/sdflib-src/src/render_engine/shaders/sdfPlane.frag
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Generating sdfPlane.frag"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfPlane.frag /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfPlane.frag

_deps/sdflib-build/sdfPlane.vert: _deps/sdflib-src/src/render_engine/shaders/sdfPlane.vert
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/Users/im-euncheol/Desktop/KIST/CBFs/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Generating sdfPlane.vert"
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && cmake -E copy_if_different /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src/src/render_engine/shaders/sdfPlane.vert /Users/im-euncheol/Desktop/KIST/CBFs/build/lib/shaders/sdfPlane.vert

copyShaders: _deps/sdflib-build/basic.frag
copyShaders: _deps/sdflib-build/basic.vert
copyShaders: _deps/sdflib-build/colors.frag
copyShaders: _deps/sdflib-build/colors.vert
copyShaders: _deps/sdflib-build/grid.frag
copyShaders: _deps/sdflib-build/grid.vert
copyShaders: _deps/sdflib-build/normals.frag
copyShaders: _deps/sdflib-build/normals.vert
copyShaders: _deps/sdflib-build/normalsSplitPlane.frag
copyShaders: _deps/sdflib-build/normalsSplitPlane.vert
copyShaders: _deps/sdflib-build/screenPlane.frag
copyShaders: _deps/sdflib-build/screenPlane.vert
copyShaders: _deps/sdflib-build/sdfOctreeLight.frag
copyShaders: _deps/sdflib-build/sdfOctreeLight.vert
copyShaders: _deps/sdflib-build/sdfOctreeMeanTrianglesPlane.frag
copyShaders: _deps/sdflib-build/sdfOctreeMeanTrianglesPlane.vert
copyShaders: _deps/sdflib-build/sdfOctreePlane.frag
copyShaders: _deps/sdflib-build/sdfOctreePlane.vert
copyShaders: _deps/sdflib-build/sdfOctreeRender.comp
copyShaders: _deps/sdflib-build/sdfPlane.frag
copyShaders: _deps/sdflib-build/sdfPlane.vert
copyShaders: _deps/sdflib-build/CMakeFiles/copyShaders.dir/build.make
.PHONY : copyShaders

# Rule to build all files generated by this target.
_deps/sdflib-build/CMakeFiles/copyShaders.dir/build: copyShaders
.PHONY : _deps/sdflib-build/CMakeFiles/copyShaders.dir/build

_deps/sdflib-build/CMakeFiles/copyShaders.dir/clean:
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build && $(CMAKE_COMMAND) -P CMakeFiles/copyShaders.dir/cmake_clean.cmake
.PHONY : _deps/sdflib-build/CMakeFiles/copyShaders.dir/clean

_deps/sdflib-build/CMakeFiles/copyShaders.dir/depend:
	cd /Users/im-euncheol/Desktop/KIST/CBFs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/im-euncheol/Desktop/KIST/CBFs /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-src /Users/im-euncheol/Desktop/KIST/CBFs/build /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build /Users/im-euncheol/Desktop/KIST/CBFs/build/_deps/sdflib-build/CMakeFiles/copyShaders.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : _deps/sdflib-build/CMakeFiles/copyShaders.dir/depend
