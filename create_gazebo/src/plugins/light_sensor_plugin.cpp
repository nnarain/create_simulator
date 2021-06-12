#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <thread>
#include <functional>
#include <cmath>
#include <memory>

static constexpr uint16_t MAX_LIGHT_VALUE = 4096;

namespace create_gazebo
{
class CreateLightRos : public gazebo::RayPlugin
{
public:
    CreateLightRos()
    {
    }

    void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf)
    {
        RayPlugin::Load(parent, sdf);

        parent_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(parent);

        if (!parent_)
        {
            gzthrow("Parent sensor must be a Ray sensor");
        }

        ros_node_ = gazebo_ros::Node::Get(sdf);

        std::string topic_name = "/light";
        if (sdf->HasElement("topicName"))
        {
            topic_name = sdf->Get<std::string>("topicName");
        }

        pub_ = ros_node_->create_publisher<std_msgs::msg::UInt16>(topic_name, rclcpp::SensorDataQoS());

        const auto world_name = parent_->WorldName();

        // Gazebo node is used to subscribe to gazebo laser scan messages
        node_ = gazebo::transport::NodePtr{new gazebo::transport::Node{}};
        node_->Init(world_name);

        scan_sub_ = node_->Subscribe(parent_->Topic(), &CreateLightRos::onScan, this);

        parent_->SetActive(false);
    }

private:
    void onConnect()
    {
        // if (!scan_sub_)
        // {
        //     ROS_INFO_STREAM_NAMED("light", "Subscring to scan messages");
        //     scan_sub_ = node_->Subscribe(parent_->Topic(), &CreateLightRos::onScan, this);
        // }
    }

    void onDisconnect()
    {
        // ROS_INFO_STREAM_NAMED("light", "Disconnecting from scan messages");
        // scan_sub_.reset();
    }

    void onScan(const ConstLaserScanStampedPtr& laser)
    {
        std_msgs::msg::UInt16 msg;

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

        pub_->publish(msg);
    }

    // Parent sensor
    gazebo::sensors::RaySensorPtr parent_;

    gazebo_ros::Node::SharedPtr ros_node_;

    // ROS publisher
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_;

    // Gazebo subscriber
    gazebo::transport::NodePtr node_;
    gazebo::transport::SubscriberPtr scan_sub_;
};

    GZ_REGISTER_SENSOR_PLUGIN(CreateLightRos)
}
