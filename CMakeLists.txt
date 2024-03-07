cmake_minimum_required(VERSION 3.8)
project(optoforce_etherdaq_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(CURL REQUIRED) # Ensure this is found, consider using a FindCURL.cmake if not automatically found
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
)

set(driver_dependencies
  rclcpp
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  tinyxml2_vendor
  TinyXML2
  std_srvs
  std_msgs
)

set(executable_driver_node etherdaq_node)
set(executable_sub etherdaq_subscriber)

set(library_driver ${executable_driver_node}_core)

# Declare a C++ library
add_library(${library_driver} SHARED
  src/etherdaq_driver.cpp
)
ament_target_dependencies(${library_driver}
  ${driver_dependencies}
)

target_link_libraries(${library_driver}
  ${Boost_LIBRARIES}
  ${CURL_LIBRARIES}
  tinyxml2::tinyxml2
)

# Declare C++ executables
add_executable(${executable_driver_node} src/etherdaq_node.cpp)
ament_target_dependencies(${executable_driver_node}
  ${driver_dependencies}
)
target_link_libraries(${executable_driver_node}
  ${library_driver}
)

add_executable(${executable_sub} src/etherdaq_subscriber.cpp)
ament_target_dependencies(${executable_sub}
  rclcpp
  geometry_msgs
  std_msgs
)

# Install targets
install(TARGETS
  ${executable_driver_node}
  ${executable_sub}
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files and other resources
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS ${library_driver}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)

ament_export_libraries(${library_driver})

# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp diagnostic_msgs diagnostic_updater geometry_msgs tinyxml2_vendor tinyxml2)

ament_package()
