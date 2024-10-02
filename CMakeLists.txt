cmake_minimum_required(VERSION 3.11)
project(segmentFromAcc)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()
# Require C++17 standard in DGtal
set(CMAKE_CXX_STANDARD 17)  
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_C_COMPILER "/usr/bin/clang")
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")

SET(AS_COMMAND_SRC
    ${PROJECT_SOURCE_DIR}/src/NormalAccumulator.cpp
)
SET(AS_TOOLS_SRC 
    ${PROJECT_SOURCE_DIR}/src/AccVoxelHelper.h
    ${PROJECT_SOURCE_DIR}/src/AccumulationSpace.h
)

# Include DGtal
SET (DGtal_DIR "/home/adam/DGtal/build")
find_package(DGtal REQUIRED) #local version installed in machine
#include_directories(${DGTAL_INCLUDE_DIRS})
#link_libraries(${DGTAL_LIBRARY_DIRS})

# Include Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)

# Include QGLViewer
SET (QGLVIEWER_INCLUDE_DIRS "/usr/include/QGLViewer")
SET (QGLVIEWER_LIBRARIES  "/usr/local/lib/libQGLViewer-qt5.so")
#find_package(QGLViewer REQUIRED)

# Create an executable
add_subdirectory(tools)
add_subdirectory(deps)
add_executable(segmentFromAcc ${AS_SOURCE_FILE} main.cpp)

# Include settings
#include("${PROJECT_BINARY_DIR}/deps/DGtal/DGtalConfig.cmake")
target_include_directories(segmentFromAcc
PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${DGTAL_INCLUDE_DIRS}
    ${QT5_INCLUDE_DIRS}
    ${QGLVIEWER_INCLUDE_DIRS}
)

# Link settings
target_link_libraries(segmentFromAcc 
PRIVATE
    ${DGTAL_LIBRARIES} 
    Qt5::Core Qt5::Widgets Qt5::OpenGL
    ${QGLVIEWER_LIBRARIES}
    polyscope
)
