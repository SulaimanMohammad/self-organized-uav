# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/sulaiman/Desktop/IMT/code/Self-organized UAV"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build"

# Include any dependencies generated for this target.
include CMakeFiles/expansion.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/expansion.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/expansion.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/expansion.dir/flags.make

CMakeFiles/expansion.dir/src/expansion.c.o: CMakeFiles/expansion.dir/flags.make
CMakeFiles/expansion.dir/src/expansion.c.o: /home/sulaiman/Desktop/IMT/code/Self-organized\ UAV/src/expansion.c
CMakeFiles/expansion.dir/src/expansion.c.o: CMakeFiles/expansion.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/expansion.dir/src/expansion.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/expansion.dir/src/expansion.c.o -MF CMakeFiles/expansion.dir/src/expansion.c.o.d -o CMakeFiles/expansion.dir/src/expansion.c.o -c "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/src/expansion.c"

CMakeFiles/expansion.dir/src/expansion.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/expansion.dir/src/expansion.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/src/expansion.c" > CMakeFiles/expansion.dir/src/expansion.c.i

CMakeFiles/expansion.dir/src/expansion.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/expansion.dir/src/expansion.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/src/expansion.c" -o CMakeFiles/expansion.dir/src/expansion.c.s

# Object files for target expansion
expansion_OBJECTS = \
"CMakeFiles/expansion.dir/src/expansion.c.o"

# External object files for target expansion
expansion_EXTERNAL_OBJECTS =

libexpansion.a: CMakeFiles/expansion.dir/src/expansion.c.o
libexpansion.a: CMakeFiles/expansion.dir/build.make
libexpansion.a: CMakeFiles/expansion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libexpansion.a"
	$(CMAKE_COMMAND) -P CMakeFiles/expansion.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/expansion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/expansion.dir/build: libexpansion.a
.PHONY : CMakeFiles/expansion.dir/build

CMakeFiles/expansion.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/expansion.dir/cmake_clean.cmake
.PHONY : CMakeFiles/expansion.dir/clean

CMakeFiles/expansion.dir/depend:
	cd "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/sulaiman/Desktop/IMT/code/Self-organized UAV" "/home/sulaiman/Desktop/IMT/code/Self-organized UAV" "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build" "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build" "/home/sulaiman/Desktop/IMT/code/Self-organized UAV/build/CMakeFiles/expansion.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/expansion.dir/depend

