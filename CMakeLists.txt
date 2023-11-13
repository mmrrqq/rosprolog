cmake_minimum_required(VERSION 3.5)
project(rosprolog)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)

endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include additional CMake macros
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(pack)
#include(plunit)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_action REQUIRED)
find_package(rcl_yaml_param_parser REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(console_bridge REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/MessageJSON.msg
  srv/JSONWrapper.srv
  DEPENDENCIES std_msgs
)

# Since ROS Humble
rosidl_get_typesupport_target(ros_cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

find_package(json_prolog_msgs REQUIRED)

# find SWI Prolog libraries
find_package(SWIPL REQUIRED)
if(NOT SWIPL_FOUND)
  message( FATAL_ERROR "swipl not found!" )
endif()

# libjson-glib-dev
# used to parse JSON strings in the C++ client
# TODO: better use RapidJSON
pkg_check_modules(JSON_GLIB REQUIRED json-glib-1.0)
add_definitions(${JSON_GLIB_CFLAGS})
link_directories(${JSON_GLIB_LIBRARIY_DIRS})

# Output C++ client libraries
set(CMAKE_CXX_FLAGS "-pthread ${CMAKE_CXX_FLAGS}")
include_directories(include
                    ${rmw_implementation_INCLUDE_DIRS})

# TODO: make shared library with PrologEngine & PrologPool
######
add_library(rosprolog_kb
	SHARED
	src/rosprolog_kb/rosprolog_kb.cpp
	src/rosprolog_kb/ros_param.cpp
	src/rosprolog_kb/ros_terms.cpp
	src/rosprolog_kb/json_ros.cpp
	src/rosprolog_kb/tf.cpp
	src/rosprolog_node/PrologEngine.cpp
	src/rosprolog_node/PrologPool.cpp)

target_link_libraries(rosprolog_kb ${SWIPL_LIBRARIES} "${ros_cpp_typesupport_target}")
ament_target_dependencies(rosprolog_kb
 rcl
 rclcpp
 console_bridge
 std_msgs
 geometry_msgs
 tf2_ros)
target_include_directories(rosprolog_kb PUBLIC ${SWIPL_INCLUDE_DIRS})

######
add_executable(rosprolog_node
	src/rosprolog_node/PrologNode.cpp
	src/rosprolog_node/PrologEngine.cpp
	src/rosprolog_node/PrologPool.cpp
)
set_target_properties(rosprolog_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

target_link_libraries(rosprolog_node ${SWIPL_LIBRARIES} rosprolog_kb)
ament_target_dependencies(rosprolog_node
  rcl
  rclcpp
  console_bridge
  json_prolog_msgs)

######
add_library(rosprolog_client
  src/rosprolog_client/PrologClient.cpp
  src/rosprolog_client/PrologQuery.cpp
  src/rosprolog_client/PrologBindings.cpp)
target_link_libraries(rosprolog_client ${JSON_GLIB_LIBRARIES} "${ros_cpp_typesupport_target}")
ament_target_dependencies(rosprolog_client
  rcl 
  rclcpp 
  json_prolog_msgs
)
target_include_directories(rosprolog_client PUBLIC ${rcl_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${json_prolog_msgs_INCLUDE_DIRS})

######
add_executable(test_rosprolog examples/test_rosprolog.cpp)
target_link_libraries(test_rosprolog rosprolog_client "${ros_cpp_typesupport_target}")

install(TARGETS rosprolog_kb
  DESTINATION lib)

install(TARGETS rosprolog_node
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS src/json_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  PROGRAMS src/rosprolog_client/rosprolog_client.py
  DESTINATION lib/${PROJECT_NAME}
)
  
FILE(GLOB sh_files "scripts/*")
install(PROGRAMS ${sh_files}
  DESTINATION share/${PROJECT_NAME}/scripts/)

# Add rosprolog_rest
install(PROGRAMS scripts/rosprolog_rest.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/rosprolog_commandline.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS ${SWIPL_LIBRARIES}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS ${SWIPL_LIBRARIES}
  DESTINATION lib
)

# Export dependencies, include for downstream/client CMake-based packages
ament_export_dependencies(rosidl_default_runtime)
ament_export_libraries(rosprolog_kb)
ament_export_include_directories(include)
ament_package(
	CONFIG_EXTRAS cmake/pack.cmake
)
