add_library(physics INTERFACE)

target_include_directories(physics INTERFACE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/collisionDetection ${CMAKE_CURRENT_SOURCE_DIR}/geometry ${CMAKE_CURRENT_SOURCE_DIR}/forces)

# library link dependencies
target_link_libraries(physics INTERFACE math geometry)
