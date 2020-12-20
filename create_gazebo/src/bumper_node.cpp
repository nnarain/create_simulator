//
// bumper_node.cpp
//
// @author Natesh Narain <nnaraindev@gmail.com>
// @date Dec 20 2020
//

#include "ros/ros.h"

#include "create_msgs/Bumper.h"
#include "gazebo_msgs/ContactsState.h"

#include "tf/tf.h"

/**
 * Convert gazebo sensors into a single create_msgs/Bumper message
*/
class Bumper
{
public:
    explicit Bumper(ros::NodeHandle& nh)
        : front_contact_threshold_{10.0}
    {
        bumper_pub_ = nh.advertise<create_msgs::Bumper>("bumper", 1, false);
        bumper_pub_timer_ = nh.createTimer(ros::Duration{1.0 / 20.0}, &Bumper::bumperPubTimerCallback, this);
        contact_sub_ = nh.subscribe("sim/bumper", 10, &Bumper::contactCallback, this);
    }

private:
    void contactCallback(const gazebo_msgs::ContactsStateConstPtr& msg)
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
                // TODO: Filter messages from a gazebo plugin?
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

    void bumperPubTimerCallback(const ros::TimerEvent&)
    {
        bumper_pub_.publish(bumper_msg_);
    }

    // Bumper message, publisher and timer
    ros::Publisher bumper_pub_;
    create_msgs::Bumper bumper_msg_;
    ros::Timer bumper_pub_timer_;
    // Subscriber for contact states
    ros::Subscriber contact_sub_;

    // Contact parameters
    double front_contact_threshold_;
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "bumper_node");

    ros::NodeHandle global_nh{""};
    ros::NodeHandle private_nh{"~"};

    Bumper bumper{global_nh};

    ros::spin();

    return 0;
}
