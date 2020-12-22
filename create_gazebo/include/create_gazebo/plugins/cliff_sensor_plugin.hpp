//
// cliff_plugin.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 21 2020
//
#ifndef CREATE_GAZEBO_CLIFF_SENSOR_PLUGIN_HPP
#define CREATE_GAZEBO_CLIFF_SENSOR_PLUGIN_HPP

#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_plugins/PubQueue.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

namespace gazebo
{
/**
 * iRobot Create Cliff Sensor Plugin
*/
// TODO: Could this be a generic cliff plugin?
class CreateCliffRos : public RayPlugin
{
public:
    CreateCliffRos();
    ~CreateCliffRos() = default;

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

    // Plugin parameters
    float min_range_;

    // ROS publisher
    PubMultiQueue pmq_;
    PubQueue<std_msgs::Bool>::Ptr pub_queue_;
    ros::Publisher pub_;

    // Gazebo subscriber
    transport::NodePtr node_;
    transport::SubscriberPtr scan_sub_;
};
}

#endif // CREATE_GAZEBO_CLIFF_SENSOR_PLUGIN_HPP
