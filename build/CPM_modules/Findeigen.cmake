include("/usr/local/lib/DGtal/Modules/CPM.cmake")
CPMAddPackage("NAME;eigen;GIT_REPOSITORY;https://gitlab.com/libeigen/eigen.git;GIT_TAG;3.4.0;DOWNLOAD_ONLY;ON")
set(eigen_FOUND TRUE)