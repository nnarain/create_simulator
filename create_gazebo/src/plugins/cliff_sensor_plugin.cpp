#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/transport/transport.hh>
// #include <gazebo_plugins/PubQueue.h>
#include <gazebo/msgs/laserscan_stamped.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

// #include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <thread>
#include <functional>
#include <memory>

namespace create_gazebo
{
class CreateCliffRos : public gazebo::RayPlugin
{
public:
    CreateCliffRos()
    {
    }

    void Load(gazebo::sensors::SensorPtr parent, sdf::ElementPtr sdf)
    {
        RayPlugin::Load(parent, sdf);

        ros_node_ = gazebo_ros::Node::Get(sdf);

        // GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
        parent_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(parent);

        if (!parent_)
        {
            gzthrow("Parent sensor must be a Ray sensor");
        }

        // Load plugin parameters
        if (!sdf->HasElement("cliffRange"))
        {
            RCLCPP_INFO_STREAM(ros_node_->get_logger(), "Cliff plugin missin parameter 'cliffRange', defaulting to 5cm");
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

        const auto world_name = parent_->WorldName();
        // const auto robot_namespace = robot_namespace(parent_, sdf, "Cliff");

        // Gazebo node is used to subscribe to gazebo laser scan messages
        node_ = gazebo::transport::NodePtr{new gazebo::transport::Node{}};
        node_->Init(world_name);

        scan_sub_ = node_->Subscribe(parent_->Topic(), &CreateCliffRos::onScan, this);

        // pmq_.startServiceThread();

        // ros::NodeHandle nh{robot_namespace};

        // ros::AdvertiseOptions opts = ros::AdvertiseOptions::create<std_msgs::Bool>(
        //     topic_name, 1,
        //     std::bind(&CreateCliffRos::onConnect, this),
        //     std::bind(&CreateCliffRos::onDisconnect, this),
        //     ros::VoidPtr{}, nullptr
        // );

        // pub_ = nh.advertise(opts);
        // pub_queue_ = pmq_.addPub<std_msgs::Bool>();

        parent_->SetActive(false);

        // ROS_INFO_STREAM_NAMED("cliff", "Starting cliff plugin: " << robot_namespace);
    }

private:
    void onConnect()
    {
        // if (!scan_sub_)
        // {
        //     ROS_INFO_STREAM_NAMED("cliff", "Subscring to scan messages");
        //     scan_sub_ = node_->Subscribe(parent_->Topic(), &CreateCliffRos::onScan, this);
        // }
    }

    void onDisconnect()
    {
        // ROS_INFO_STREAM_NAMED("cliff", "Disconnecting from scan messages");
        // scan_sub_.reset();
    }

    void onScan(const ConstLaserScanStampedPtr& laser)
    {
        std_msgs::msg::Bool msg;
        msg.data = laser->scan().ranges(0) > min_range_;

        pub_->publish(msg);
    }

    // Parent sensor
    gazebo::sensors::RaySensorPtr parent_;
    // ROS node
    gazebo_ros::Node::SharedPtr ros_node_;

    // // Plugin parameters
    float min_range_{0.0f};

    // // ROS publisher
    // PubMultiQueue pmq_;
    // PubQueue<std_msgs::Bool>::Ptr pub_queue_;
    // ros::Publisher pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;

    // Gazebo subscriber
    gazebo::transport::NodePtr node_;
    gazebo::transport::SubscriberPtr scan_sub_;

};

    GZ_REGISTER_SENSOR_PLUGIN(CreateCliffRos)
}
