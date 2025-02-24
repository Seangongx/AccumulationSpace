cmake_minimum_required(VERSION 3.11)
project(segmentFromAcc)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()

# Require C++17 standard in DGtal
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# Debug settings
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(WITH_LIBIGL OFF)
SET(WITH_CGAL OFF)
SET(WITH_GMP OFF)
include(dgtal)
include(polyscope)

SET(AS_COMMAND_SRC
    ${PROJECT_SOURCE_DIR}/src/NormalAccumulator.cpp
)
SET(AS_TOOLS_SRC 
    ${PROJECT_SOURCE_DIR}/src/AccumulationAlgorithms.h
    ${PROJECT_SOURCE_DIR}/src/AccumulationAlgorithms.ipp
    ${PROJECT_SOURCE_DIR}/src/AccumulationSpace.h
    ${PROJECT_SOURCE_DIR}/src/AccumulationSpace.ipp
    ${PROJECT_SOURCE_DIR}/src/PolyscopeEnvironment.h
    ${PROJECT_SOURCE_DIR}/src/PolyscopeEnvironment.ipp
)

# Create an executable
add_subdirectory(tools)
add_executable(segmentFromAcc ${AS_SOURCE_FILE} main.cpp)

# Include settings
target_include_directories(segmentFromAcc
PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${DGTAL_INCLUDE_DIRS}
)

# Link settings
target_link_libraries(segmentFromAcc 
PRIVATE
    ${DGTAL_LIBRARIES}
    polyscope
)