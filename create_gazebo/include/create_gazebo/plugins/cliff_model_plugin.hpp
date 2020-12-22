//
// cliff_model_plugin.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 21 2020
//
#ifndef CREATE_GAZEBO_CLIFF_MODEL_PLUGIN_HPP
#define CREATE_GAZEBO_CLIFF_MODEL_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <create_msgs/Cliff.h>

namespace gazebo
{
/**
 * Model plugin that consolidates individual cliff sensor sim topics into a single
 * /cliff topic that would be emitted by the ROS driver
*/
class CreateCliffModelRos : public ModelPlugin
{
public:
    CreateCliffModelRos();
    ~CreateCliffModelRos() = default;

    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

private:
    /**
     * Cliff sensor callback
    */
    void sensorCallback(const std_msgs::BoolConstPtr& msg, bool& field);

    /**
     * 
    */
    void timerCallback(const ros::TimerEvent&);

    // Main cliff message publisher
    ros::Publisher cliff_pub_;
    // Cliff message
    create_msgs::Cliff cliff_msg_;
    // Publish timer
    ros::Timer pub_timer_;

    // Subscribers for individual sensors
    ros::Subscriber left_sub_;
    ros::Subscriber leftfront_sub_;
    ros::Subscriber right_sub_;
    ros::Subscriber rightfront_sub_;

    // Individual sensor states
    bool left_state_;
    bool leftfront_state_;
    bool right_state_;
    bool rightfront_state_;
};
}

#endif // CREATE_GAZEBO_CLIFF_MODEL_PLUGIN_HPP
