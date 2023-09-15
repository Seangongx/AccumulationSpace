CMAKE_MINIMUM_REQUIRED(VERSION 3.11)

PROJECT(AccSpaceExe)


#SET(AS_SOURCE_FILE 
#${PROJECT_SOURCE_DIR}/src/GeodesicGraphComputer.cpp 
#${PROJECT_SOURCE_DIR}/src/NormalAccumulator.cpp
#)


include_directories ("${PROJECT_SOURCE_DIR}/src")


### Required in DGtal
message(STATUS "-------------------------------------------------------------------------------")
message(STATUS "Checking if DGtal is installed:")
FIND_PACKAGE(DGtal REQUIRED)
INCLUDE_DIRECTORIES(${DGTAL_INCLUDE_DIRS})
LINK_DIRECTORIES(${DGTAL_LIBRARY_DIRS})
message(STATUS "DGtal found.")

#ADD_EXECUTABLE(AccSpaceExe ${AS_SOURCE_FILE} main.cpp)
ADD_EXECUTABLE(AccSpaceExe main.cpp)
TARGET_LINK_LIBRARIES(AccSpaceExe ${DGTAL_LIBRARIES})