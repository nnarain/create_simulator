cmake_minimum_required(VERSION 3.0.2)
project(create_gazebo)

add_compile_options(-std=c++17 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
    create_msgs
    roscpp
    gazebo_msgs
    tf
)

catkin_package(
    CATKIN_DEPENDS
        create_msgs
        roscpp
        gazebo_msgs
        tf
)

add_executable(bumper_node
    src/bumper_node.cpp
)
target_include_directories(bumper_node
    PRIVATE
        include/
        ${catkin_INCLUDE_DIRS}
)
target_link_libraries(bumper_node
    ${catkin_LIBRARIES}
)

install(TARGETS bumper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
