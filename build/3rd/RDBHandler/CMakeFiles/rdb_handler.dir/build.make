# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ivtest/WTC/cmakelist

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ivtest/WTC/cmakelist/build

# Include any dependencies generated for this target.
include 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/compiler_depend.make

# Include the progress variables for this target.
include 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/progress.make

# Include the compile flags for this target's objects.
include 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/flags.make

3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.o: 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/flags.make
3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.o: ../3rd/RDBHandler/RDBHandler.cc
3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.o: 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ivtest/WTC/cmakelist/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.o"
	cd /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.o -MF CMakeFiles/rdb_handler.dir/RDBHandler.cc.o.d -o CMakeFiles/rdb_handler.dir/RDBHandler.cc.o -c /home/ivtest/WTC/cmakelist/3rd/RDBHandler/RDBHandler.cc

3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rdb_handler.dir/RDBHandler.cc.i"
	cd /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ivtest/WTC/cmakelist/3rd/RDBHandler/RDBHandler.cc > CMakeFiles/rdb_handler.dir/RDBHandler.cc.i

3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rdb_handler.dir/RDBHandler.cc.s"
	cd /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ivtest/WTC/cmakelist/3rd/RDBHandler/RDBHandler.cc -o CMakeFiles/rdb_handler.dir/RDBHandler.cc.s

# Object files for target rdb_handler
rdb_handler_OBJECTS = \
"CMakeFiles/rdb_handler.dir/RDBHandler.cc.o"

# External object files for target rdb_handler
rdb_handler_EXTERNAL_OBJECTS =

3rd/RDBHandler/librdb_handler.a: 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/RDBHandler.cc.o
3rd/RDBHandler/librdb_handler.a: 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/build.make
3rd/RDBHandler/librdb_handler.a: 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ivtest/WTC/cmakelist/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library librdb_handler.a"
	cd /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler && $(CMAKE_COMMAND) -P CMakeFiles/rdb_handler.dir/cmake_clean_target.cmake
	cd /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rdb_handler.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
3rd/RDBHandler/CMakeFiles/rdb_handler.dir/build: 3rd/RDBHandler/librdb_handler.a
.PHONY : 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/build

3rd/RDBHandler/CMakeFiles/rdb_handler.dir/clean:
	cd /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler && $(CMAKE_COMMAND) -P CMakeFiles/rdb_handler.dir/cmake_clean.cmake
.PHONY : 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/clean

3rd/RDBHandler/CMakeFiles/rdb_handler.dir/depend:
	cd /home/ivtest/WTC/cmakelist/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ivtest/WTC/cmakelist /home/ivtest/WTC/cmakelist/3rd/RDBHandler /home/ivtest/WTC/cmakelist/build /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler /home/ivtest/WTC/cmakelist/build/3rd/RDBHandler/CMakeFiles/rdb_handler.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : 3rd/RDBHandler/CMakeFiles/rdb_handler.dir/depend
