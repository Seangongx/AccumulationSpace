if (TARGET DGtal)
  return()
endif()

include(FetchContent)

message(STATUS "Fetching DGtal")

SET(BUILD_EXAMPLES OFF)

FetchContent_Declare(
    DGtal
    GIT_REPOSITORY https://github.com/DGtal-team/DGtal.git
    GIT_TAG cdccf7aa53d9e651fc93db7a5f22e8bdf1313d0b
    )
FetchContent_MakeAvailable(DGtal)

include("${DGtal_BINARY_DIR}/DGtalConfig.cmake")
include_directories("${DGTAL_INCLUDE_DIRS}")