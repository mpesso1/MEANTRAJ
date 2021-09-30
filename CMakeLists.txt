cmake_minimum_required(VERSION 3.0.0)

project(GPMP_Motion_Planner)

# Note that it is also possible to use add_subdirectory() to additional CMakeLists.txt files that define the below code in their respected subdirectories.
#This may be convienient if the directories are getting long and things are getting messy.


# Define variable that defines the location and file of the header file
add_library(library library/gpmp/gpmp.cpp library/gpmp/gpmp.h)
target_include_directories(library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/library/gpmp")
#include the directory path to the search alg for finding the files

# Define variable that defines the location and file of an executable
add_executable(gpmp_run source/run.cpp)
target_link_libraries(gpmp_run PUBLIC library)
#link the defined header files with the running file that located in different directories