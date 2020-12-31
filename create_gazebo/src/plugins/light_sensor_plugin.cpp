#include "create_gazebo/plugins/light_sensor_plugin.hpp"

#include <gazebo_plugins/gazebo_ros_utils.h>

#include <thread>
#include <functional>
#include <cmath>

static constexpr uint16_t MAX_LIGHT_VALUE = 4096;

namespace gazebo
{
    GZ_REGISTER_SENSOR_PLUGIN(CreateLightRos)

    CreateLightRos::CreateLightRos()
    {
    }

    void CreateLightRos::Load(sensors::SensorPtr parent, sdf::ElementPtr sdf)
    {
        ROS_INFO_STREAM_NAMED("light", "loading light plugin");
        RayPlugin::Load(parent, sdf);

        parent_ = std::dynamic_pointer_cast<sensors::RaySensor>(parent);

        if (!parent_)
        {
            gzthrow("Parent sensor must be a Ray sensor");
        }

        std::string topic_name = "/light";
        if (sdf->HasElement("topicName"))
        {
            topic_name = sdf->Get<std::string>("topicName");
        }

        // Setup ROS
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("light", "ROS is not initialized, cannot load plugin");
            return;
        }

        const auto world_name = parent_->WorldName();
        const auto robot_namespace = GetRobotNamespace(parent_, sdf, "light");

        // Gazebo node is used to subscribe to gazebo laser scan messages
        node_ = transport::NodePtr{new transport::Node{}};
        node_->Init(world_name);

        pmq_.startServiceThread();

        ros::NodeHandle nh{robot_namespace};

        ros::AdvertiseOptions opts = ros::AdvertiseOptions::create<std_msgs::UInt16>(
            topic_name, 1,
            std::bind(&CreateLightRos::onConnect, this),
            std::bind(&CreateLightRos::onDisconnect, this),
            ros::VoidPtr{}, nullptr
        );

        pub_ = nh.advertise(opts);
        pub_queue_ = pmq_.addPub<std_msgs::UInt16>();

        parent_->SetActive(false);

        ROS_INFO_STREAM_NAMED("light", "Starting light plugin: " << robot_namespace);
    }

    void CreateLightRos::onConnect()
    {
        if (!scan_sub_)
        {
            ROS_INFO_STREAM_NAMED("light", "Subscring to scan messages");
            scan_sub_ = node_->Subscribe(parent_->Topic(), &CreateLightRos::onScan, this);
        }
    }

    void CreateLightRos::onDisconnect()
    {
        ROS_INFO_STREAM_NAMED("light", "Disconnecting from scan messages");
        scan_sub_.reset();
    }

    void CreateLightRos::onScan(const ConstLaserScanStampedPtr& laser)
    {
        std_msgs::UInt16 msg;

        const auto distance = laser->scan().ranges(0);

        // Check if there is no obstacle within the sensor range
        if (std::isinf(distance))
        {
            msg.data = 0;
        }
        else
        {
            const auto ratio = distance / laser->scan().range_max();
            msg.data = MAX_LIGHT_VALUE * (1.0 - ratio);
        }

        pub_queue_->push(msg, pub_);
    }
}
