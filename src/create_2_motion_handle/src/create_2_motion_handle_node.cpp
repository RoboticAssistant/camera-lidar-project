#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <publisher_subscriber/pub_sub.hpp>
#include <strings.h>

// Create a subscribe function which deals with the data received
void direction_subscribe(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("create_2_motion_handle: Data Received: [%s]", msg->data.c_str());
}


int main(int argc, char *argv[])
{
    // Check for the connectivity of this module with the pub sub module

    // Create a publisher object
    if(!strcmp(argv[1], "talker"))
    {
        string send_data(argv[2]);
        cout << "Talker" << endl;
        Pub_Sub publisher(argc, argv, "talker");

        while(1)
        {
            // Publishing data
            publisher.publisher_data(send_data, "test_topic");
        }
    }
    else if(!strcmp(argv[1], "listener"))
    {
        cout << "Listener" << endl;
        Pub_Sub subscriber(argc, argv, "listener");
        subscriber.subscribe_data(&direction_subscribe, "test_topic");
    }

    // Create a subscriber object

    return 0;
}
