#include "create_gazebo/plugins/cliff_sensor_plugin.hpp"

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <thread>
#include <functional>

namespace gazebo
{
    GZ_REGISTER_SENSOR_PLUGIN(CreateCliffRos)

    CreateCliffRos::CreateCliffRos()
    {
    }

    void CreateCliffRos::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
    {
        ROS_INFO_STREAM_NAMED("cliff", "loading cliff plugin");
        RayPlugin::Load(parent, sdf);

        GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
        parent_ = dynamic_pointer_cast<sensors::RaySensor>(parent);

        if (!parent_)
        {
            gzthrow("Parent sensor must be a Ray sensor");
        }

        // Load plugin parameters
        if (!sdf->HasElement("cliffRange"))
        {
            ROS_INFO_NAMED("cliff", "Cliff plugin missin parameter 'cliffRange', defaulting to 5cm");
            min_range_ = 0.05;
        }
        else
        {
            min_range_ = sdf->Get<double>("cliffRange");
        }

        std::string topic_name = "/cliff";
        if (sdf->HasElement("topicName"))
        {
            topic_name = sdf->Get<std::string>("topicName");
        }

        // Setup ROS
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("cliff", "ROS is not initialized, cannot load plugin");
            return;
        }

        const auto world_name = parent_->WorldName();
        const auto robot_namespace = GetRobotNamespace(parent_, sdf, "Cliff");

        // Gazebo node is used to subscribe to gazebo laser scan messages
        node_ = transport::NodePtr{new transport::Node{}};
        node_->Init(world_name);

        pmq_.startServiceThread();

        ros::NodeHandle nh{robot_namespace};

        ros::AdvertiseOptions opts = ros::AdvertiseOptions::create<std_msgs::Bool>(
            topic_name, 1,
            std::bind(&CreateCliffRos::onConnect, this),
            std::bind(&CreateCliffRos::onDisconnect, this),
            ros::VoidPtr{}, nullptr
        );

        pub_ = nh.advertise(opts);
        pub_queue_ = pmq_.addPub<std_msgs::Bool>();

        parent_->SetActive(false);

        ROS_INFO_STREAM_NAMED("cliff", "Starting cliff plugin: " << robot_namespace);
    }

    void CreateCliffRos::onConnect()
    {
        if (!scan_sub_)
        {
            ROS_INFO_STREAM_NAMED("cliff", "Subscring to scan messages");
            scan_sub_ = node_->Subscribe(parent_->Topic(), &CreateCliffRos::onScan, this);
        }
    }

    void CreateCliffRos::onDisconnect()
    {
        ROS_INFO_STREAM_NAMED("cliff", "Disconnecting from scan messages");
        scan_sub_.reset();
    }

    void CreateCliffRos::onScan(const ConstLaserScanStampedPtr& laser)
    {
        std_msgs::Bool msg;
        msg.data = laser->scan().ranges(0) > min_range_;

        pub_queue_->push(msg, pub_);
    }
}
