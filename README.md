# Normal Accumulation Space and Confidence on surface mesh.

## Dependencies
1. CMake >= 3.11 
2. DGtal 1.4 git@github.com:DGtal-team/DGtal.git
3. polyscope git@github.com:nmwsharp/polyscope.git

## How to install 
1. Git clone this project from the Github
2. Configure and build this project by CMake

You will obtain the following display: 

<img width="868" alt="Interface" src="https://github.com/kerautret/ExpeAccSegment/samples/Interface.png">

## Miscellaneous

### How to debug in VSCode
1. If you use VSCode, keep the .vscode folder for the project
2. Use <CMake Tools> extension to help you build the project and initiate the Debug Env. Check Debugging below
3. Stop writing the task.json in VSCode to initiate Debugging (Modern C++ POV)

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