cmake_minimum_required(VERSION 3.8)
project(demo_gnc_sensor_abstraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED) 
# find_package(gazebo_ros REQUIRED) 
#find_package(tinyxml2_vendor REQUIRED) 
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#find_package(tinyxml2 REQUIRED)
# Locate tinyxml2 manually
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h)
find_library(TINYXML2_LIBRARY tinyxml2)

if (NOT TINYXML2_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find tinyxml2.h")
endif()

if (NOT TINYXML2_LIBRARY)
  message(FATAL_ERROR "Could not find the tinyxml2 library")
endif()

include_directories(${TINYXML2_INCLUDE_DIR})


add_executable(demo_sensor src/demo_sensor.cpp) 
#ament_target_dependencies(demo_sensor rclcpp gazebo_ros tinyxml2) 

target_link_libraries(demo_sensor ${TINYXML2_LIBRARY})

ament_target_dependencies(demo_sensor rclcpp std_msgs ament_index_cpp) 

install(TARGETS
  demo_sensor
  DESTINATION lib/${PROJECT_NAME}) 
  
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch) 
 
install(DIRECTORY sensors/
  DESTINATION share/${PROJECT_NAME}/sensors)

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

ament_package()
