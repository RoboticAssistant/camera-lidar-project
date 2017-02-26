#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <publisher_subscriber/pub_sub.hpp>

// Create a subscribe function which deals with the data received
void direction_subscribe(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("create_2_motion_handle: Data Received: [%s]", msg->data.c_str());
}


int main()
{
    // Check for the connectivity of this module with the pub sub module

    // Create a publisher object
    Pub_Sub();

    // Create a subscriber object

    return 0;
}
