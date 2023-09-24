CMAKE_MINIMUM_REQUIRED(VERSION 3.11)

PROJECT(AccSpaceExe)

SET(AS_SOURCE_FILE 
    ${PROJECT_SOURCE_DIR}/src/GeodesicGraphComputer.cpp 
    ${PROJECT_SOURCE_DIR}/src/NormalAccumulator.cpp
)

### Required in DGtal
FIND_PACKAGE(DGtal REQUIRED)
#INCLUDE_DIRECTORIES(${DGTAL_INCLUDE_DIRS})
#LINK_DIRECTORIES(${DGTAL_LIBRARY_DIRS})

### Required in Qt5
find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)
#find_package(QGLViewer REQUIRED)

### Required in QGLViewer
SET (QGLVIEWER_INCLUDE_DIRS "/usr/include/QGLViewer")
SET (QGLVIEWER_LIBRARIES  "/usr/local/lib/libQGLViewer-qt5.so")
#find_package(QGLViewer REQUIRED)

### Generate Project
add_subdirectory(bin)
ADD_EXECUTABLE(AccSpaceExe ${AS_SOURCE_FILE} main.cpp)
### include directories
target_include_directories(AccSpaceExe
PRIVATE
    ${PROJECT_SOURCE_DIR}/src
    ${DGTAL_INCLUDE_DIRS}
    ${QT5_INCLUDE_DIRS}
    ${QGLVIEWER_INCLUDE_DIRS}
)

### link directories
TARGET_LINK_LIBRARIES(AccSpaceExe 
PRIVATE
    ${DGTAL_LIBRARIES} 
    Qt5::Core Qt5::Widgets Qt5::OpenGL
    ${QGLVIEWER_LIBRARIES})
