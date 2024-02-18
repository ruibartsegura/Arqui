cmake_minimum_required(VERSION 3.8)
project(practica3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kobuki_ros_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
    rclcpp
    kobuki_ros_interfaces
    geometry_msgs
    tf2_geometry_msgs
    tf2
    tf2_ros
)
include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/practica3/TfNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(tf_main src/tf_main.cpp)
ament_target_dependencies(tf_main ${dependencies})
target_link_libraries(tf_main ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  tf_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
