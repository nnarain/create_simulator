
#include <gazebo/common/Plugin.hh>

#include <gazebo_msgs/msg/contacts_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <create_msgs/msg/bumper.hpp>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>


#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <functional>

// #include <ros/ros.h>
// #include <create_msgs/Bumper.h>
// #include <nav_msgs/Odometry.h>
// #include <gazebo_msgs/ContactsState.h>
// #include <std_msgs/UInt16.h>

// #include <tf/tf.h>



namespace create_gazebo_plugins
{
class CreateBumperModelRos : public gazebo::ModelPlugin
{
public:
    CreateBumperModelRos()
        // , heading_{1, 0, 0}
    {
    }

    void Load(gazebo::physics::ModelPtr, sdf::ElementPtr sdf)
    {
        ros_node_ = gazebo_ros::Node::Get(sdf);

        std::string topic_name{"/bumper"};
        if (sdf->HasElement("topicName"))
        {
            topic_name = sdf->Get<std::string>("topicName");
        }

        bumper_pub_ = ros_node_->create_publisher<create_msgs::msg::Bumper>(topic_name, rclcpp::SensorDataQoS());

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "[Bumper Model Plugin]: Topic name: " << topic_name);

        // Load parameters
        double updateRate = 10.0;
        if (sdf->HasElement("updateRate"))
        {
            updateRate = sdf->Get<double>("updateRate");
        }

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "[Bumper Model Plugin]: Update Rate: " << updateRate << " Hz");

        const auto millis = static_cast<unsigned int>((1.0 / updateRate) * 1000);

        bumper_pub_timer_ = ros_node_->create_wall_timer(
                                            std::chrono::milliseconds{millis},
                                            std::bind(&CreateBumperModelRos::bumperPubTimerCallback, this)
                                       );

        std::string odom_topic{"/odom"};
        if (sdf->HasElement("odomTopic"))
        {
            odom_topic = sdf->Get<std::string>("odomTopic");
        }

        if (sdf->HasElement("frontContactThreshold"))
        {
            front_contact_threshold_ = sdf->Get<double>("frontContactThreshold");
        }

        RCLCPP_INFO_STREAM(ros_node_->get_logger(), "[Bumper Model Plugin]: Front Contact Threshold: " << front_contact_threshold_);

        // Setup sensor subscriber
        // contact_sub_ = nh.subscribe("sim/bumper", 10, &CreateBumperModelRos::sensorCallback, this);
        contact_sub_ = ros_node_->create_subscription<gazebo_msgs::msg::ContactsState>("sim/bumper", 10,
                                    std::bind(&CreateBumperModelRos::sensorCallback, this, std::placeholders::_1));

        // // Odometry callback
        // odom_sub_ = nh.subscribe(odom_topic, 10, &CreateBumperModelRos::odomCallback, this);
        odom_sub_ = ros_node_->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                                std::bind(&CreateBumperModelRos::odomCallback, this, std::placeholders::_1));


        // Light sensor subscribers
        using LightSensorCallback = std::function<void(std_msgs::msg::UInt16::UniquePtr)>;
        auto callback_factory = [&](uint16_t& field) -> LightSensorCallback {
            return [&](std_msgs::msg::UInt16::UniquePtr msg) -> void { field = msg->data; };
        };

        light_subs_[0] = ros_node_->create_subscription<std_msgs::msg::UInt16>("/sim/light_left",        1, callback_factory(bumper_msg_.light_signal_left));
        light_subs_[1] = ros_node_->create_subscription<std_msgs::msg::UInt16>("/sim/light_frontleft",   1, callback_factory(bumper_msg_.light_signal_front_left));
        light_subs_[2] = ros_node_->create_subscription<std_msgs::msg::UInt16>("/sim/light_centerleft",  1, callback_factory(bumper_msg_.light_signal_center_left));
        light_subs_[3] = ros_node_->create_subscription<std_msgs::msg::UInt16>("/sim/light_centerright", 1, callback_factory(bumper_msg_.light_signal_center_right));
        light_subs_[4] = ros_node_->create_subscription<std_msgs::msg::UInt16>("/sim/light_frontright",  1, callback_factory(bumper_msg_.light_signal_front_right));
        light_subs_[5] = ros_node_->create_subscription<std_msgs::msg::UInt16>("/sim/light_right",       1, callback_factory(bumper_msg_.light_signal_right));
    }

private:
    void sensorCallback(gazebo_msgs::msg::ContactsState::UniquePtr msg)
    {
        bumper_msg_.is_left_pressed = false;
        bumper_msg_.is_right_pressed = false;

        for (const auto& state : msg->states)
        {
            for (const auto& normal : state.contact_normals)
            {
                tf2::Vector3 n;
                tf2::fromMsg(normal, n);

                // Contact points are on a cylindrical collision body.
                // Therefore, the contact normals are all pointed inwards to the robot's center (base_link origin)

                // Invert the normal (point outwards from origin)
                n *= -1.0;

                tf2::Vector3 x{1.0, 0.0, 0.0};
                // Check if the contact comes from the front of the robot
                if (heading_.dot(n) >= 0.0)
                {
                    if (x.angle(n) <= tf2Radians(front_contact_threshold_))
                    {
                        bumper_msg_.is_left_pressed = true;
                        bumper_msg_.is_right_pressed = true;
                    }
                    else if (n.y() > 0.0)
                    {
                        // left
                        bumper_msg_.is_left_pressed = true;
                        bumper_msg_.is_right_pressed = false;
                    }
                    else
                    {
                        // right
                        bumper_msg_.is_left_pressed = false;
                        bumper_msg_.is_right_pressed = true;
                    }
                }
            }
        }
    }

    void odomCallback(nav_msgs::msg::Odometry::UniquePtr msg)
    {
        tf2::Quaternion r;
        tf2::fromMsg(msg->pose.pose.orientation, r);

        const tf2::Vector3 d{1, 0, 0};
        heading_ = tf2::quatRotate(r, d);
    }

    void bumperPubTimerCallback()
    {
        bumper_pub_->publish(bumper_msg_);
    }

    gazebo_ros::Node::SharedPtr ros_node_;

    rclcpp::Publisher<create_msgs::msg::Bumper>::SharedPtr bumper_pub_;
    create_msgs::msg::Bumper bumper_msg_;

    rclcpp::TimerBase::SharedPtr bumper_pub_timer_;


    // Subscriber for contact states
    rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr contact_sub_;
    // Odom to get robot heading
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Light sensors
    rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr light_subs_[6];

    // Contact parameters
    double front_contact_threshold_{10.0};

    // Robot heading
    tf2::Vector3 heading_{1, 0, 0};
};


GZ_REGISTER_MODEL_PLUGIN(CreateBumperModelRos)
}
