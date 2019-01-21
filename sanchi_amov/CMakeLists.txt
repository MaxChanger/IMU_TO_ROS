cmake_minimum_required(VERSION 2.8.3)
project(sanchi_amov)


add_definitions(-std=c++11)
find_package(catkin REQUIRED roscpp sensor_msgs tf cmake_modules)

find_package(catkin REQUIRED COMPONENTS)


include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)

catkin_package(
)

add_executable(sanchi_amov
  src/sanchi_amov.cc
)

target_link_libraries(sanchi_amov
  ${catkin_LIBRARIES}
)
