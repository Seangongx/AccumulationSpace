# Normal Accumulation Space and Confidence on surface mesh

## Dependencies
1. CMake >= 3.11 
2. DGtal 1.4 git@github.com:DGtal-team/DGtal.git
3. polyscope git@github.com:nmwsharp/polyscope.git
4. Boost required in DGtal

## How to install 
1. Git clone this project from the Github
2. Configure and build this project by CMake in [Your Project Path]/build
3. Make program

You will obtain the following display: 

![polyAccEdit Interface](https://github.com/Seangongx/AccumulationSpace/blob/main/samples/Interface.png)

## Miscellaneous
Polyscope said: "On Ubuntu and friends, you may want to `apt-get install xorg-dev libglu1-mesa-dev freeglut3-dev mesa-common-dev` to pull graphics and windowing related headers to build."
If you has GPU vendor error on mesa, look at this [discussion](https://github.com/microsoft/WSL/issues/12412).

``` sudo apt install ppa-purge
sudo ppa-purge ppa:kisak/kisak-mesa
MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA glxinfo -B
name of display: :0
...
OpenGL vendor string: Microsoft Corporation
OpenGL renderer string: D3D12 (NVIDIA GeForce RTX 4070)
```

### How to debug in VSCode
1. If you use VSCode, keep the .vscode folder for the project
2. Use <CMake Tools> extension to help you build the project and initiate the Debug Env. Check Debugging below
3. Stop writing the task.json in VSCode to initiate Debugging (Modern C++ POV)
4. If it exists DGtal problem: Try 1) Git clone the DGtal library and build (Remove the old path in CMakeLists.txt) and 2)Configure and build this project by CMake adding DGtal_DIR= [Your DGtal Library Path]/build

#### Warning: if you use Ubuntu OS and VSCode, please install VSCode from website rather than Snapd.
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