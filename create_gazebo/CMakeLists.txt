cmake_minimum_required(VERSION 3.5)
project(create_gazebo)

add_compile_options(-std=c++17 -Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(create_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(gazebo REQUIRED)

include_directories(include
  ${gazebo_dev_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${image_transport_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

#-----------------------------------------------------------------------------------------------------------------------
# Gazebo plugins
#-----------------------------------------------------------------------------------------------------------------------
link_directories(${gazebo_dev_LIBRARY_DIRS})

# Bumper plugin
add_library(create_bumper_model_plugin SHARED
    src/plugins/bumper_model_plugin.cpp
)
ament_target_dependencies(create_bumper_model_plugin
    "gazebo_dev"
    "rclcpp"
    "gazebo_ros"
    "create_msgs"
    "std_msgs"
    "nav_msgs"
)
ament_export_libraries(create_bumper_model_plugin)

# Cliff sensor plugin
add_library(create_cliff_sensor_plugin
    src/plugins/cliff_sensor_plugin.cpp
)
ament_target_dependencies(create_cliff_sensor_plugin
    "gazebo_dev"
    "rclcpp"
    "gazebo_ros"
    "std_msgs"
)
ament_export_libraries(create_cliff_sensor_plugin)

# Cliff Model Plugin
# add_library(create_cliff_model_plugin
#     src/plugins/cliff_model_plugin.cpp
# )
# ament_target_dependencies(create_cliff_model_plugin
#     "gazebo_dev"
#     "rclcpp"
#     "gazebo_ros"
#     "std_msgs"
# )
# ament_export_libraries(create_cliff_model_plugin)

# Light sensor plugin
add_library(create_light_sensor_plugin
    src/plugins/light_sensor_plugin.cpp
)
ament_target_dependencies(create_light_sensor_plugin
    "gazebo_dev"
    "rclcpp"
    "gazebo_ros"
    "std_msgs"
)
ament_export_libraries(create_light_sensor_plugin)

#-----------------------------------------------------------------------------------------------------------------------
# Installs
#-----------------------------------------------------------------------------------------------------------------------
install(
    TARGETS
        create_bumper_model_plugin
        create_cliff_sensor_plugin
        # create_cliff_model_plugin # TODO(nnarain): Add cliff sensor message then Port to ROS2
        create_light_sensor_plugin
    DESTINATION
        lib/${PROJECT_NAME}
)

install(
    DIRECTORY
        launch
        worlds
        models
        maps
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
