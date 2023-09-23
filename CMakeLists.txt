CMAKE_MINIMUM_REQUIRED(VERSION 3.11)

PROJECT(AccSpaceExe)

SET(AS_SOURCE_FILE 
    ${PROJECT_SOURCE_DIR}/src/GeodesicGraphComputer.cpp 
    ${PROJECT_SOURCE_DIR}/src/NormalAccumulator.cpp
)

### Required in DGtal
message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "Checking if DGtal is installed:")
FIND_PACKAGE(DGtal REQUIRED)
INCLUDE_DIRECTORIES(${DGTAL_INCLUDE_DIRS})
LINK_DIRECTORIES(${DGTAL_LIBRARY_DIRS})
message(STATUS "DGtal found.")

#ADD_EXECUTABLE(AccSpaceExe ${AS_SOURCE_FILE} main.cpp)

add_subdirectory(bin)
ADD_EXECUTABLE(AccSpaceExe main.cpp)
target_include_directories(AccSpaceExe
PUBLIC
 ${PROJECT_SOURCE_DIR}/src
 ${PROJECT_SOURCE_DIR}/ext
)
TARGET_LINK_LIBRARIES(AccSpaceExe ${DGTAL_LIBRARIES})