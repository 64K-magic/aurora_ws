cmake_minimum_required(VERSION 3.8)
project(pointcloud_to_gridmap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

cmake_policy(SET CMP0074 NEW)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ament_auto_add_executable(pointcloud_to_gridmap_node
  src/pointcloud_to_gridmap_node.cpp
)

target_link_libraries(pointcloud_to_gridmap_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(TARGETS pointcloud_to_gridmap_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_auto_package()