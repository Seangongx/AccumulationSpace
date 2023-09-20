# The project about Accumulation Space and Confidence computation on triangle mesh model.

## Dependencies
CMake >= 2.6  
PCL Library for the compAccFromSDP tool  
DGtal (eventually with QGLViewer to have 3D display tools: illustrationGraphAllSteps and   testGeodesicGraphDisplay)  
DGtalTools (optional) : provide 3dVolViewer and 3dSDPViewer tools  
DGtalTools-contrib (optional) : provide graphViewer tool  

## How to install 
1. Please follow the short tutorial below to install DGtal (DGtalTools and DGtalTools-contrib are optional)
2. Git clone this project and use CMake to generate it

## How to debug
1. If you use VSCode, keep the .vscode folder for the project
2. Use <CMake Tools> extension to help you build the project and initiate the Debug Env. Check Debugging below

## Short guide for installation all the dependencies of DGtal(take Ubuntu for instance)

### ZLIB (DGtal dependency)
Execute `sudo apt install zlib1g-dev`

### CMake (source code)
1. Execute `sudo apt-get install build-essential` if lack of compiler
2. Execute `sudo apt-get install libssl-dev` if ssh error occurs
3. Download CMake and execute `tar -zxvf cmake-x.xx.x.tar.gz` then enter into the folder
4. Execute `./bootstrap` in current directory and `make [-j4]` `sudo make install`

### GMP (DGtal dependency)
Execute `sudo apt-get install libgmp-dev`

### Cairo (DGtal dependency)
Execute `sudo apt-get install libcairo2-dev`

### ITK (DGtal dependency)
1. Execute `sudo apt install libeigen3-dev` to install required Eigen library ahead
2. Follow the Guide in following link can easily install ITK with CMake through source code
```https://itk.org/download/```
Please read the chapter 2.CONFIGURING AND BUILDING ITK in guide book  

### libQGLViewer (DGtalTool dependency)
Execute `sudo apt-get install libqglviewer-dev-qt5`

### Boost (Please follow the official document to install)

### Git (optional)

## DGtal and tools install
After installing all the dependencies, you can use git or download zip file from github to install DGtal
`git clone git@github.com:DGtal-team/DGtal.git`
`git clone git@github.com:DGtal-team/DGtalTools.git`
`git clone git@github.com:DGtal-team/DGtalTools-contrib.git`
1. Execute `mkdir build; cd build; cmake(ccmake) ..` to configure and generate the project.
2. Execute `make XXX` to generate the XXX program or tool you want, or install it `sudo make install XXX`

## Debugging
#### Warning: if you use Ubuntu OS and VSCode, please install VSCode from website rather than Snapd.
#### Suggest: if you use VSCode, please use <CMake Tools> extenstion to help you configure debug env
#### Stop writing the task.json in VSCode to initiate Debugging (Modern C++ POV)

### Qt(source code for debugging)

1. Execute `sudo apt-get install mesa-common-dev` to install OpenGL dependencies before install Qt5
2. Execute `sudo apt install libxcb-xinerama0 libxcb-cursor0` to avoid the error occurring before install Qt5
`./qt-unified-linux-x64-4.6.0-online.run: error while loading shared libraries: libxcb-xinerama.so.0: cannot open shared object file: No such file or directory
s`
3. Execute `chmod +x qt-unified-linux-x64-x.x.x-online.run` switch the authority and execute .run file

#### If you meet the error in VSCode installed in Snapd:
`symbol lookup error: /snap/core20/current/lib/x86_64-linux-gnu/libpthread.so.0: undefined symbol: __libc_pthread_init, version GLIBC_PRIVATE`  
lead to the program running in the Ubuntu terminal but fail in VSCode Bash  
change the settings of VSCode Bash  
```
"terminal.integrated.env.linux": {
        //"QT_QPA_PLATFORM" : "xcb",
        "GTK_PATH": ""
    }
```