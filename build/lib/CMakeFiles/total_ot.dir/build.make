# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/powdersnow/workspace/C_derived/object_tracking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/powdersnow/workspace/C_derived/object_tracking/build

# Include any dependencies generated for this target.
include lib/CMakeFiles/total_ot.dir/depend.make

# Include the progress variables for this target.
include lib/CMakeFiles/total_ot.dir/progress.make

# Include the compile flags for this target's objects.
include lib/CMakeFiles/total_ot.dir/flags.make

lib/CMakeFiles/total_ot.dir/customer.cpp.o: lib/CMakeFiles/total_ot.dir/flags.make
lib/CMakeFiles/total_ot.dir/customer.cpp.o: ../lib/customer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object lib/CMakeFiles/total_ot.dir/customer.cpp.o"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/total_ot.dir/customer.cpp.o -c /home/powdersnow/workspace/C_derived/object_tracking/lib/customer.cpp

lib/CMakeFiles/total_ot.dir/customer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/total_ot.dir/customer.cpp.i"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/powdersnow/workspace/C_derived/object_tracking/lib/customer.cpp > CMakeFiles/total_ot.dir/customer.cpp.i

lib/CMakeFiles/total_ot.dir/customer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/total_ot.dir/customer.cpp.s"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/powdersnow/workspace/C_derived/object_tracking/lib/customer.cpp -o CMakeFiles/total_ot.dir/customer.cpp.s

lib/CMakeFiles/total_ot.dir/detect.cpp.o: lib/CMakeFiles/total_ot.dir/flags.make
lib/CMakeFiles/total_ot.dir/detect.cpp.o: ../lib/detect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object lib/CMakeFiles/total_ot.dir/detect.cpp.o"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/total_ot.dir/detect.cpp.o -c /home/powdersnow/workspace/C_derived/object_tracking/lib/detect.cpp

lib/CMakeFiles/total_ot.dir/detect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/total_ot.dir/detect.cpp.i"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/powdersnow/workspace/C_derived/object_tracking/lib/detect.cpp > CMakeFiles/total_ot.dir/detect.cpp.i

lib/CMakeFiles/total_ot.dir/detect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/total_ot.dir/detect.cpp.s"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/powdersnow/workspace/C_derived/object_tracking/lib/detect.cpp -o CMakeFiles/total_ot.dir/detect.cpp.s

lib/CMakeFiles/total_ot.dir/hungarian.cpp.o: lib/CMakeFiles/total_ot.dir/flags.make
lib/CMakeFiles/total_ot.dir/hungarian.cpp.o: ../lib/hungarian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object lib/CMakeFiles/total_ot.dir/hungarian.cpp.o"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/total_ot.dir/hungarian.cpp.o -c /home/powdersnow/workspace/C_derived/object_tracking/lib/hungarian.cpp

lib/CMakeFiles/total_ot.dir/hungarian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/total_ot.dir/hungarian.cpp.i"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/powdersnow/workspace/C_derived/object_tracking/lib/hungarian.cpp > CMakeFiles/total_ot.dir/hungarian.cpp.i

lib/CMakeFiles/total_ot.dir/hungarian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/total_ot.dir/hungarian.cpp.s"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/powdersnow/workspace/C_derived/object_tracking/lib/hungarian.cpp -o CMakeFiles/total_ot.dir/hungarian.cpp.s

lib/CMakeFiles/total_ot.dir/iou.cpp.o: lib/CMakeFiles/total_ot.dir/flags.make
lib/CMakeFiles/total_ot.dir/iou.cpp.o: ../lib/iou.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object lib/CMakeFiles/total_ot.dir/iou.cpp.o"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/total_ot.dir/iou.cpp.o -c /home/powdersnow/workspace/C_derived/object_tracking/lib/iou.cpp

lib/CMakeFiles/total_ot.dir/iou.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/total_ot.dir/iou.cpp.i"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/powdersnow/workspace/C_derived/object_tracking/lib/iou.cpp > CMakeFiles/total_ot.dir/iou.cpp.i

lib/CMakeFiles/total_ot.dir/iou.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/total_ot.dir/iou.cpp.s"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/powdersnow/workspace/C_derived/object_tracking/lib/iou.cpp -o CMakeFiles/total_ot.dir/iou.cpp.s

lib/CMakeFiles/total_ot.dir/kalman_filter.cpp.o: lib/CMakeFiles/total_ot.dir/flags.make
lib/CMakeFiles/total_ot.dir/kalman_filter.cpp.o: ../lib/kalman_filter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object lib/CMakeFiles/total_ot.dir/kalman_filter.cpp.o"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/total_ot.dir/kalman_filter.cpp.o -c /home/powdersnow/workspace/C_derived/object_tracking/lib/kalman_filter.cpp

lib/CMakeFiles/total_ot.dir/kalman_filter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/total_ot.dir/kalman_filter.cpp.i"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/powdersnow/workspace/C_derived/object_tracking/lib/kalman_filter.cpp > CMakeFiles/total_ot.dir/kalman_filter.cpp.i

lib/CMakeFiles/total_ot.dir/kalman_filter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/total_ot.dir/kalman_filter.cpp.s"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/powdersnow/workspace/C_derived/object_tracking/lib/kalman_filter.cpp -o CMakeFiles/total_ot.dir/kalman_filter.cpp.s

lib/CMakeFiles/total_ot.dir/tracker.cpp.o: lib/CMakeFiles/total_ot.dir/flags.make
lib/CMakeFiles/total_ot.dir/tracker.cpp.o: ../lib/tracker.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object lib/CMakeFiles/total_ot.dir/tracker.cpp.o"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/total_ot.dir/tracker.cpp.o -c /home/powdersnow/workspace/C_derived/object_tracking/lib/tracker.cpp

lib/CMakeFiles/total_ot.dir/tracker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/total_ot.dir/tracker.cpp.i"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/powdersnow/workspace/C_derived/object_tracking/lib/tracker.cpp > CMakeFiles/total_ot.dir/tracker.cpp.i

lib/CMakeFiles/total_ot.dir/tracker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/total_ot.dir/tracker.cpp.s"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/powdersnow/workspace/C_derived/object_tracking/lib/tracker.cpp -o CMakeFiles/total_ot.dir/tracker.cpp.s

# Object files for target total_ot
total_ot_OBJECTS = \
"CMakeFiles/total_ot.dir/customer.cpp.o" \
"CMakeFiles/total_ot.dir/detect.cpp.o" \
"CMakeFiles/total_ot.dir/hungarian.cpp.o" \
"CMakeFiles/total_ot.dir/iou.cpp.o" \
"CMakeFiles/total_ot.dir/kalman_filter.cpp.o" \
"CMakeFiles/total_ot.dir/tracker.cpp.o"

# External object files for target total_ot
total_ot_EXTERNAL_OBJECTS =

lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/customer.cpp.o
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/detect.cpp.o
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/hungarian.cpp.o
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/iou.cpp.o
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/kalman_filter.cpp.o
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/tracker.cpp.o
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/build.make
lib/libtotal_ot.a: lib/CMakeFiles/total_ot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/powdersnow/workspace/C_derived/object_tracking/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libtotal_ot.a"
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/total_ot.dir/cmake_clean_target.cmake
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/total_ot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lib/CMakeFiles/total_ot.dir/build: lib/libtotal_ot.a

.PHONY : lib/CMakeFiles/total_ot.dir/build

lib/CMakeFiles/total_ot.dir/clean:
	cd /home/powdersnow/workspace/C_derived/object_tracking/build/lib && $(CMAKE_COMMAND) -P CMakeFiles/total_ot.dir/cmake_clean.cmake
.PHONY : lib/CMakeFiles/total_ot.dir/clean

lib/CMakeFiles/total_ot.dir/depend:
	cd /home/powdersnow/workspace/C_derived/object_tracking/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/powdersnow/workspace/C_derived/object_tracking /home/powdersnow/workspace/C_derived/object_tracking/lib /home/powdersnow/workspace/C_derived/object_tracking/build /home/powdersnow/workspace/C_derived/object_tracking/build/lib /home/powdersnow/workspace/C_derived/object_tracking/build/lib/CMakeFiles/total_ot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lib/CMakeFiles/total_ot.dir/depend

