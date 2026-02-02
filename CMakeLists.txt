cmake_minimum_required(VERSION 3.8)
project(mockamap)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 指定源文件（不使用 GLOB，避免编译不需要的文件）
set(${PROJECT_NAME}_SRCS
  src/mockamap.cpp
  src/maps.cpp
  src/perlinnoise.cpp
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS})

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
