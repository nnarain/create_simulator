//
// bumper_model_plugin.hpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 22 2020
//
#ifndef CREATE_GAZEBO_BUMPER_MODEL_PLUGIN_HPP
#define CREATE_GAZEBO_BUMPER_MODEL_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <create_msgs/Bumper.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ContactsState.h>
#include <std_msgs/UInt16.h>

#include <tf/tf.h>

namespace gazebo
{
/**
 * Model plugin that consolidates bumper sensor states into a single create_msgs/Bumper message
*/
class CreateBumperModelRos : public ModelPlugin
{
public:
    CreateBumperModelRos();
    ~CreateBumperModelRos() = default;

    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

private:
    /**
     * Contact sensor callback
    */
    void sensorCallback(const gazebo_msgs::ContactsStateConstPtr& msg);

    /**
     * Odom callback
    */
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);

    /**
     * Light sensor callback
    */
    void lightSensorCallback(const std_msgs::UInt16ConstPtr& msg, uint16_t& field);

    /**
     * Update bumper publisher
    */
    void bumperPubTimerCallback(const ros::TimerEvent&);

    // Bumper message, publisher and timer
    ros::Publisher bumper_pub_;
    create_msgs::Bumper bumper_msg_;
    ros::Timer bumper_pub_timer_;
    // Subscriber for contact states
    ros::Subscriber contact_sub_;
    // Odom to get robot heading
    ros::Subscriber odom_sub_;
    // Light sensors
    ros::Subscriber light_subs_[6];

    // Contact parameters
    double front_contact_threshold_;

    // Robot heading
    tf::Vector3 heading_;
};
}

#endif // CREATE_GAZEBO_BUMPER_MODEL_PLUGIN_HPP
