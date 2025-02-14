if (TARGET polyscope)
  return()
endif()

include(FetchContent)

message(STATUS "Fetching polyscope")

FetchContent_Declare(
    polyscope
    GIT_REPOSITORY https://github.com/nmwsharp/polyscope.git
    GIT_TAG        0615cd790d7966b081a8778d1b9d49da3ef05ea1
    GIT_SHALLOW    TRUE
    )
FetchContent_MakeAvailable(polyscope)