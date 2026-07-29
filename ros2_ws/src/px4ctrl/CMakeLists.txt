cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 包含目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIRS}
)

# 把这个cpp文件编译为可执行文件
add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlParam.cpp
  src/input.cpp
  src/controller.cpp
  src/PX4CtrlFSM.cpp
)

# 加入编译所需要的依赖
ament_target_dependencies(px4ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  Eigen3
)

# 把目标安装到对应的目录，便于以后的查找和安装
install(TARGETS
  px4ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
