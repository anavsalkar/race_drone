cmake_minimum_required(VERSION 2.8.3)
project(rpg_quadrotor_integration_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
find_package(rostest REQUIRED)
find_package(flightlib REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(${flightlib_INCLUDE_DIRS})

add_rostest_gtest(
  rpg_quadrotor_integration_test 
  launch/rpg_quadrotor_integration_test.launch 
  src/rpg_quadrotor_integration_test.cpp)
target_link_libraries(rpg_quadrotor_integration_test ${catkin_LIBRARIES})

cs_add_executable(my_racing_node src/my_racing.cpp)
target_link_libraries(my_racing_node ${catkin_LIBRARIES})


cs_install()
cs_export()
