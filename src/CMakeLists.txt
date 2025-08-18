add_library(physics INTERFACE)

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

target_include_directories(physics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/collisionDetection ${CMAKE_CURRENT_SOURCE_DIR}/geometry ${CMAKE_CURRENT_SOURCE_DIR}/forces)

# library link dependencies
target_link_libraries(physics INTERFACE math geometry)
