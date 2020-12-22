#include "create_gazebo/plugins/bumper_model_plugin.hpp"

#include "tf/tf.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(CreateBumperModelRos)

CreateBumperModelRos::CreateBumperModelRos()
    : front_contact_threshold_{10.0}
{
}

void CreateBumperModelRos::Load(physics::ModelPtr, sdf::ElementPtr sdf)
{
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = nullptr;

        ros::init(argc, argv, "create_bumper_model_plugin");
    }

    // Load parameters
    double updateRate = 10.0;
    if (sdf->HasElement("updateRate"))
    {
        updateRate = sdf->Get<double>("updateRate");
    }

    ROS_INFO_STREAM("[Bumper Model Plugin]: Update Rate: " << updateRate << " Hz");

    std::string topic_name{"/bumper"};
    if (sdf->HasElement("topicName"))
    {
        topic_name = sdf->Get<std::string>("topicName");
    }

    ROS_INFO_STREAM("[Bumper Model Plugin]: Update Rate: " << topic_name);

    if (sdf->HasElement("frontContactThreshold"))
    {
        front_contact_threshold_ = sdf->Get<double>("frontContactThreshold");
    }

    ROS_INFO_STREAM("[Bumper Model Plugin]: Front Contact Threshold: " << front_contact_threshold_);

    // Setup publishers
    ros::NodeHandle nh;

    bumper_pub_ = nh.advertise<create_msgs::Bumper>(topic_name, 1);
    bumper_pub_timer_ = nh.createTimer(ros::Duration{1.0 / updateRate}, &CreateBumperModelRos::bumperPubTimerCallback, this);

    // Setup sensor subscriber
    contact_sub_ = nh.subscribe("sim/bumper", 10, &CreateBumperModelRos::sensorCallback, this);
}

void CreateBumperModelRos::sensorCallback(const gazebo_msgs::ContactsStateConstPtr& msg)
{
    bumper_msg_.is_left_pressed = false;
    bumper_msg_.is_right_pressed = false;

    for (const auto& state : msg->states)
    {
        for (const auto& normal : state.contact_normals)
        {
            tf::Vector3 n;
            tf::vector3MsgToTF(normal, n);

            // Contact points are on a cylindrical collision body.
            // Therefore, the contact normals are all pointed inwards to the robot's center (base_link origin)

            // Invert the normal (point outwards from origin)
            n *= -1.0;

            tf::Vector3 x{1.0, 0.0, 0.0};
            // Check if the contact comes from the front of the robot
            if (x.dot(n) > 0.0)
            {
                // TODO: Handle contact front pressing both bumpers
                if (x.angle(n) <= tfRadians(front_contact_threshold_))
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

void CreateBumperModelRos::bumperPubTimerCallback(const ros::TimerEvent&)
{
    bumper_pub_.publish(bumper_msg_);
}

}
