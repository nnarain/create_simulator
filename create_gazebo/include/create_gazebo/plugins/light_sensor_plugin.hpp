//
// light_sensor_plugin.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 21 2020
//
#ifndef CREATE_GAZEBO_LIGHT_SENSOR_PLUGIN_HPP
#define CREATE_GAZEBO_LIGHT_SENSOR_PLUGIN_HPP

#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/PubQueue.h>

#include <ros/ros.h>
#include <std_msgs/UInt16.h>

namespace gazebo
{
/**
 * iRobot Create Light Sensor Plugin
*/
class CreateLightRos : public RayPlugin
{
public:
    CreateLightRos();
    ~CreateLightRos() = default;

    /**
     * Load plugin
    */
    void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf);

private:
    /**
     * Topic connect callback
    */
    void onConnect();
    /**
     * Topic disconnect calback
    */
    void onDisconnect();
    /**
     * Ray scan callback
    */
    void onScan(const ConstLaserScanStampedPtr& msg);

    // Parent sensor
    sensors::RaySensorPtr parent_;

    // ROS publisher
    PubMultiQueue pmq_;
    PubQueue<std_msgs::UInt16>::Ptr pub_queue_;
    ros::Publisher pub_;

    // Gazebo subscriber
    transport::NodePtr node_;
    transport::SubscriberPtr scan_sub_;
};
}

#endif // CREATE_GAZEBO_LIGHT_SENSOR_PLUGIN_HPP
