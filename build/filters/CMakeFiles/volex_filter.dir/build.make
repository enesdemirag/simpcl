# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/enes/GitHub/zed_filtering/src/filters

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/enes/GitHub/zed_filtering/build/filters

# Include any dependencies generated for this target.
include CMakeFiles/volex_filter.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/volex_filter.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/volex_filter.dir/flags.make

CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o: CMakeFiles/volex_filter.dir/flags.make
CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o: /home/enes/GitHub/zed_filtering/src/filters/src/volex_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/enes/GitHub/zed_filtering/build/filters/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o -c /home/enes/GitHub/zed_filtering/src/filters/src/volex_filter.cpp

CMakeFiles/volex_filter.dir/src/volex_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/volex_filter.dir/src/volex_filter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/enes/GitHub/zed_filtering/src/filters/src/volex_filter.cpp > CMakeFiles/volex_filter.dir/src/volex_filter.cpp.i

CMakeFiles/volex_filter.dir/src/volex_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/volex_filter.dir/src/volex_filter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/enes/GitHub/zed_filtering/src/filters/src/volex_filter.cpp -o CMakeFiles/volex_filter.dir/src/volex_filter.cpp.s

CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.requires:

.PHONY : CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.requires

CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.provides: CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.requires
	$(MAKE) -f CMakeFiles/volex_filter.dir/build.make CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.provides.build
.PHONY : CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.provides

CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.provides.build: CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o


# Object files for target volex_filter
volex_filter_OBJECTS = \
"CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o"

# External object files for target volex_filter
volex_filter_EXTERNAL_OBJECTS =

volex_filter: CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o
volex_filter: CMakeFiles/volex_filter.dir/build.make
volex_filter: CMakeFiles/volex_filter.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/enes/GitHub/zed_filtering/build/filters/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable volex_filter"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/volex_filter.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/volex_filter.dir/build: volex_filter

.PHONY : CMakeFiles/volex_filter.dir/build

CMakeFiles/volex_filter.dir/requires: CMakeFiles/volex_filter.dir/src/volex_filter.cpp.o.requires

.PHONY : CMakeFiles/volex_filter.dir/requires

CMakeFiles/volex_filter.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/volex_filter.dir/cmake_clean.cmake
.PHONY : CMakeFiles/volex_filter.dir/clean

CMakeFiles/volex_filter.dir/depend:
	cd /home/enes/GitHub/zed_filtering/build/filters && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/enes/GitHub/zed_filtering/src/filters /home/enes/GitHub/zed_filtering/src/filters /home/enes/GitHub/zed_filtering/build/filters /home/enes/GitHub/zed_filtering/build/filters /home/enes/GitHub/zed_filtering/build/filters/CMakeFiles/volex_filter.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/volex_filter.dir/depend

