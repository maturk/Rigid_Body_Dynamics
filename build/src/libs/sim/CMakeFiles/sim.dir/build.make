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
CMAKE_SOURCE_DIR = /home/maturk/git/CMM/a5-maturk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/maturk/git/CMM/a5-maturk/build

# Include any dependencies generated for this target.
include src/libs/sim/CMakeFiles/sim.dir/depend.make

# Include the progress variables for this target.
include src/libs/sim/CMakeFiles/sim.dir/progress.make

# Include the compile flags for this target's objects.
include src/libs/sim/CMakeFiles/sim.dir/flags.make

src/libs/sim/CMakeFiles/sim.dir/src/RB.cpp.o: src/libs/sim/CMakeFiles/sim.dir/flags.make
src/libs/sim/CMakeFiles/sim.dir/src/RB.cpp.o: ../src/libs/sim/src/RB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maturk/git/CMM/a5-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/libs/sim/CMakeFiles/sim.dir/src/RB.cpp.o"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sim.dir/src/RB.cpp.o -c /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RB.cpp

src/libs/sim/CMakeFiles/sim.dir/src/RB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sim.dir/src/RB.cpp.i"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RB.cpp > CMakeFiles/sim.dir/src/RB.cpp.i

src/libs/sim/CMakeFiles/sim.dir/src/RB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sim.dir/src/RB.cpp.s"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RB.cpp -o CMakeFiles/sim.dir/src/RB.cpp.s

src/libs/sim/CMakeFiles/sim.dir/src/RBRenderer.cpp.o: src/libs/sim/CMakeFiles/sim.dir/flags.make
src/libs/sim/CMakeFiles/sim.dir/src/RBRenderer.cpp.o: ../src/libs/sim/src/RBRenderer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maturk/git/CMM/a5-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/libs/sim/CMakeFiles/sim.dir/src/RBRenderer.cpp.o"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sim.dir/src/RBRenderer.cpp.o -c /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RBRenderer.cpp

src/libs/sim/CMakeFiles/sim.dir/src/RBRenderer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sim.dir/src/RBRenderer.cpp.i"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RBRenderer.cpp > CMakeFiles/sim.dir/src/RBRenderer.cpp.i

src/libs/sim/CMakeFiles/sim.dir/src/RBRenderer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sim.dir/src/RBRenderer.cpp.s"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RBRenderer.cpp -o CMakeFiles/sim.dir/src/RBRenderer.cpp.s

src/libs/sim/CMakeFiles/sim.dir/src/RBUtils.cpp.o: src/libs/sim/CMakeFiles/sim.dir/flags.make
src/libs/sim/CMakeFiles/sim.dir/src/RBUtils.cpp.o: ../src/libs/sim/src/RBUtils.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/maturk/git/CMM/a5-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/libs/sim/CMakeFiles/sim.dir/src/RBUtils.cpp.o"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sim.dir/src/RBUtils.cpp.o -c /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RBUtils.cpp

src/libs/sim/CMakeFiles/sim.dir/src/RBUtils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sim.dir/src/RBUtils.cpp.i"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RBUtils.cpp > CMakeFiles/sim.dir/src/RBUtils.cpp.i

src/libs/sim/CMakeFiles/sim.dir/src/RBUtils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sim.dir/src/RBUtils.cpp.s"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && /usr/bin/x86_64-linux-gnu-g++-9 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/maturk/git/CMM/a5-maturk/src/libs/sim/src/RBUtils.cpp -o CMakeFiles/sim.dir/src/RBUtils.cpp.s

# Object files for target sim
sim_OBJECTS = \
"CMakeFiles/sim.dir/src/RB.cpp.o" \
"CMakeFiles/sim.dir/src/RBRenderer.cpp.o" \
"CMakeFiles/sim.dir/src/RBUtils.cpp.o"

# External object files for target sim
sim_EXTERNAL_OBJECTS =

src/libs/sim/libsim.a: src/libs/sim/CMakeFiles/sim.dir/src/RB.cpp.o
src/libs/sim/libsim.a: src/libs/sim/CMakeFiles/sim.dir/src/RBRenderer.cpp.o
src/libs/sim/libsim.a: src/libs/sim/CMakeFiles/sim.dir/src/RBUtils.cpp.o
src/libs/sim/libsim.a: src/libs/sim/CMakeFiles/sim.dir/build.make
src/libs/sim/libsim.a: src/libs/sim/CMakeFiles/sim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/maturk/git/CMM/a5-maturk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX static library libsim.a"
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && $(CMAKE_COMMAND) -P CMakeFiles/sim.dir/cmake_clean_target.cmake
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/libs/sim/CMakeFiles/sim.dir/build: src/libs/sim/libsim.a

.PHONY : src/libs/sim/CMakeFiles/sim.dir/build

src/libs/sim/CMakeFiles/sim.dir/clean:
	cd /home/maturk/git/CMM/a5-maturk/build/src/libs/sim && $(CMAKE_COMMAND) -P CMakeFiles/sim.dir/cmake_clean.cmake
.PHONY : src/libs/sim/CMakeFiles/sim.dir/clean

src/libs/sim/CMakeFiles/sim.dir/depend:
	cd /home/maturk/git/CMM/a5-maturk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/maturk/git/CMM/a5-maturk /home/maturk/git/CMM/a5-maturk/src/libs/sim /home/maturk/git/CMM/a5-maturk/build /home/maturk/git/CMM/a5-maturk/build/src/libs/sim /home/maturk/git/CMM/a5-maturk/build/src/libs/sim/CMakeFiles/sim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/libs/sim/CMakeFiles/sim.dir/depend
