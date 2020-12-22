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
        create_bumper_model_plugin
        create_cliff_sensor_plugin
)

#-----------------------------------------------------------------------------------------------------------------------
# Gazebo plugins
#-----------------------------------------------------------------------------------------------------------------------
link_directories(${GAZEBO_LIBRARY_DIRS})

# Bumper plugin
add_library(create_bumper_model_plugin
    src/plugins/bumper_model_plugin.cpp
)
target_include_directories(create_bumper_model_plugin
    PUBLIC
        include/
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(create_bumper_model_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin
)
add_dependencies(create_bumper_model_plugin ${catkin_EXPORTED_TARGETS})

# Cliff sensor plugin
add_library(create_cliff_sensor_plugin
    src/plugins/cliff_sensor_plugin.cpp
)
target_include_directories(create_cliff_sensor_plugin
    PUBLIC
        include/
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(create_cliff_sensor_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} RayPlugin
)
add_dependencies(create_cliff_sensor_plugin ${catkin_EXPORTED_TARGETS})

# Cliff Model Plugin
add_library(create_cliff_model_plugin
    src/plugins/cliff_model_plugin.cpp
)
target_include_directories(create_cliff_model_plugin
    PUBLIC
        include/
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(create_cliff_model_plugin
    ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}
)
add_dependencies(create_cliff_model_plugin ${catkin_EXPORTED_TARGETS})

#-----------------------------------------------------------------------------------------------------------------------
# Installs
#-----------------------------------------------------------------------------------------------------------------------
install(
    TARGETS
        create_cliff_sensor_plugin
        create_cliff_model_plugin
    DESTINATION         ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
