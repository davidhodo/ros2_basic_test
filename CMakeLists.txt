cmake_minimum_required(VERSION 3.5)
project(basic_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(component)



add_library(comp_sub component/sub.cpp)
ament_target_dependencies(comp_sub rclcpp std_msgs sensor_msgs OpenCV)


add_library(comp_pub component/pub.cpp)
ament_target_dependencies(comp_pub rclcpp std_msgs sensor_msgs OpenCV)


add_executable(composition src/composition.cpp)
ament_target_dependencies(composition rclcpp std_msgs sensor_msgs)
target_link_libraries(composition comp_sub comp_pub)

add_executable(standalone_pub src/standalone_pub.cpp)
ament_target_dependencies(standalone_pub rclcpp std_msgs sensor_msgs)
target_link_libraries(standalone_pub comp_pub)

add_executable(standalone_sub src/standalone_sub.cpp)
ament_target_dependencies(standalone_sub rclcpp std_msgs sensor_msgs)
target_link_libraries(standalone_sub comp_sub)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  composition
  comp_sub
  comp_pub
  standalone_pub
  standalone_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
