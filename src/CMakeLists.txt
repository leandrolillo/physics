add_library(${LIBRARY_NAME} INTERFACE)

FetchContent_Declare(
    math
    GIT_REPOSITORY https://github.com/leandrolillo/math.git
    GIT_TAG master
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(math)

FetchContent_Declare(
    geometry
    GIT_REPOSITORY https://github.com/leandrolillo/geometry.git
    GIT_TAG master
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(geometry)

add_subdirectory(physics)
add_subdirectory(forces)

# library link dependencies
target_link_libraries(${LIBRARY_NAME} INTERFACE math geometry)
