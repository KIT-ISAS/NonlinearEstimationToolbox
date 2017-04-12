
#######################################
# Project Settings
#######################################

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(NonlinearEstimationToolbox)

set(TOOLBOX_MAJOR_VERSION 1)
set(TOOLBOX_MINOR_VERSION 5)
set(TOOLBOX_PATCH_VERSION 1)

set(TOOLBOX_FOLDER_NAME "nonlinear-estimation-toolbox")

SET(CMAKE_BUILD_TYPE Release)

SET(BUILD_SHARED_LIBS ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

#######################################
# Set Compiler Flags
#######################################

SET(COMPILE_FLAGS "-Wall -std=c++11 -Wfatal-errors -fopenmp")

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${COMPILE_FLAGS}")

SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${COMPILE_FLAGS}")

INCLUDE_DIRECTORIES(
    "${CMAKE_SOURCE_DIR}/Library/"
    "${CMAKE_SOURCE_DIR}/../Matlab/Toolbox/Mex/Include/"
)

#######################################
# Project Structure
#######################################

ADD_SUBDIRECTORY(Library)

ADD_SUBDIRECTORY(Examples)

# Build and install toolbox pkg-config file
CONFIGURE_FILE(
    "${PROJECT_SOURCE_DIR}/nonlinear-estimation-toolbox.pc.in"
    "${PROJECT_BINARY_DIR}/nonlinear-estimation-toolbox.pc"
)

INSTALL(
    FILES "${PROJECT_BINARY_DIR}/nonlinear-estimation-toolbox.pc"    
    DESTINATION "lib/pkgconfig"
)

