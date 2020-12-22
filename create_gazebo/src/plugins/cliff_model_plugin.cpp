#include "create_gazebo/plugins/cliff_model_plugin.hpp"

#include <functional>

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(CreateCliffModelRos)

CreateCliffModelRos::CreateCliffModelRos()
{
}

void CreateCliffModelRos::Load(physics::ModelPtr, sdf::ElementPtr sdf)
{
    if (!ros::isInitialized())
    {
        int argc = 0;
        char** argv = nullptr;

        ros::init(argc, argv, "create_cliff_model_plugin");
    }

    // Load parameters
    double updateRate = 10.0;
    if (sdf->HasElement("updateRate"))
    {
        updateRate = sdf->Get<double>("updateRate");
    }

    ROS_INFO_STREAM("[Cliff Model Plugin]: Update Rate: " << updateRate);

    std::string topic_name{"/cliff"};
    if (sdf->HasElement("topicName"))
    {
        topic_name = sdf->Get<std::string>("topicName");
    }

    ROS_INFO_STREAM("[Cliff Model Plugin]: Topic name: " << topic_name);

    ros::NodeHandle nh;

    // Setup publisher
    cliff_pub_ = nh.advertise<create_msgs::Cliff>(topic_name, 1);
    pub_timer_ = nh.createTimer(ros::Duration{1.0 / updateRate}, &CreateCliffModelRos::timerCallback, this);

    // Setup subscribers
    using namespace boost::placeholders;

    left_sub_ = nh.subscribe<std_msgs::Bool>("/sim/left_cliff", 1,
                             boost::bind(&CreateCliffModelRos::sensorCallback, this, _1, boost::ref(left_state_)));
    right_sub_ = nh.subscribe<std_msgs::Bool>("/sim/right_cliff", 1,
                             boost::bind(&CreateCliffModelRos::sensorCallback, this, _1, boost::ref(right_state_)));
    leftfront_sub_ = nh.subscribe<std_msgs::Bool>("/sim/leftfront_cliff", 1,
                             boost::bind(&CreateCliffModelRos::sensorCallback, this, _1, boost::ref(leftfront_state_)));
    rightfront_sub_ = nh.subscribe<std_msgs::Bool>("/sim/rightfront_cliff", 1,
                             boost::bind(&CreateCliffModelRos::sensorCallback, this, _1, boost::ref(rightfront_state_)));
}

void CreateCliffModelRos::sensorCallback(const std_msgs::BoolConstPtr& msg, bool& field)
{
    field = msg->data;
}

void CreateCliffModelRos::timerCallback(const ros::TimerEvent&)
{
    cliff_msg_.header.stamp = ros::Time::now();
    cliff_msg_.header.seq++;

    cliff_msg_.is_cliff_left = left_state_;
    cliff_msg_.is_cliff_front_left = leftfront_state_;
    cliff_msg_.is_cliff_right = right_state_;
    cliff_msg_.is_cliff_front_right = rightfront_state_;

    cliff_pub_.publish(cliff_msg_);
}

}