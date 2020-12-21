cmake_minimum_required(VERSION 3.0.2)
project(create_gazebo)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
    create_msgs
    roscpp
    gazebo_msgs
    gazebo_plugins
    gazebo_ros
    tf
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS
        create_msgs
        roscpp
        gazebo_msgs
        gazebo_plugins
        gazebo_ros
        tf
    LIBRARIES
        create_cliff_plugin
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

#-----------------------------------------------------------------------------------------------------------------------
# Gazebo plugins
#-----------------------------------------------------------------------------------------------------------------------
link_directories(${GAZEBO_LIBRARY_DIRS})

# TODO: Bumper plugin

# Cliff sensor plugin
add_library(create_cliff_plugin
    src/plugins/cliff_plugin.cpp
)
target_include_directories(create_cliff_plugin
    PUBLIC
        include/
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(create_cliff_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin
)
add_dependencies(create_cliff_plugin ${catkin_EXPORTED_TARGETS})

#-----------------------------------------------------------------------------------------------------------------------
# Installs
#-----------------------------------------------------------------------------------------------------------------------
install(TARGETS bumper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS create_cliff_plugin
    DESTINATION         ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
