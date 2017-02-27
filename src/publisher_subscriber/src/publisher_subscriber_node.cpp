#include <iostream>
#include <ros/ros.h>
#include <publisher_subscriber/pub_sub.hpp>
#include <std_msgs/String.h>

// This project is mainly used for the publishing and subsribing of data.
// Everywhere in the project, you have to use this publisher and subscriber thing.
// There is no need to create any special pub sub models in your classes.

void callback_temp(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Data Received: [%s]", msg->data.c_str());
}

int main(int argc, char *argv[])
{
    if(!strcmp(argv[1], "talker"))
    {
        string send_data(argv[2]);
        cout << "Talker" << endl;

        msg_details publish_details;
        publish_details.is_talker = true;
        publish_details.is_listener = false;
        publish_details.pub_repeat = true;
        publish_details.frequency = 10;
        publish_details.received_callback = callback_temp;

        Pub_Sub publisher(argc, argv, publish_details);

        while(1)
        {
            // Publishing data
            publisher.publisher_data(send_data);
        }
    }
    else if(!strcmp(argv[1], "listener"))
    {
        cout << "Listener" << endl;

        msg_details subscribe_details;
        subscribe_details.is_talker = false;
        subscribe_details.is_listener = true;
        subscribe_details.pub_repeat = true;
        subscribe_details.frequency = 10;

        Pub_Sub subscriber(argc, argv, subscribe_details);
        subscriber.subscribe_data(&callback_temp);
    }

    return 0;
}
