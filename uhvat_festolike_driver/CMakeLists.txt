cmake_minimum_required(VERSION 3.0.2)
project(uhvat_festolike_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    message_generation
    dynamixel_sdk
)

# add_message_files(
#   FILES
# )

add_service_files(
  FILES
    SetGripperState.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    dynamixel_sdk
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

