#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <publisher_subscriber/pub_sub.hpp>
#include <strings.h>

// Create a subscribe function which deals with the data received
void direction_subscribe(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("\ncreate_2_motion_handle: Data Received: [%s]", msg->data.c_str());
}


int main(int argc, char *argv[])
{
    string motion("++++++++--------");

    // Check for the connectivity of this module with the pub sub module

    // Create a publisher object
    if(!strcmp(argv[1], "talker"))
    {
        string send_data(argv[2]);
        cout << "Talker" << endl;

        msg_details publish_details;
        publish_details.is_talker = true;
        publish_details.is_listener = false;
        publish_details.pub_repeat = true;
        publish_details.frequency = 1;
        publish_details.message_topic = "test";

        Pub_Sub publisher(argc, argv, publish_details);

        ros::Rate loop_rate(publish_details.frequency);
        while(ros::ok())
        {
            // Publishing data
            publisher.publisher_data(send_data);
            loop_rate.sleep();
        }
    }
    else if(!strcmp(argv[1], "listener"))
    {
        cout << "Listener" << endl;

        msg_details subscribe_details;
        subscribe_details.is_talker = false;
        subscribe_details.is_listener = true;
        subscribe_details.pub_repeat = false;
        subscribe_details.message_topic = "test";
        subscribe_details.received_callback = direction_subscribe;

        Pub_Sub subscriber(argc, argv, subscribe_details);
        // Subscribe to topic
        subscriber.subscribe_data(&direction_subscribe);
    }

    // Depending upon the received data being a subscriber,

    return 0;
}
