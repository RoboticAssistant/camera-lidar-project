#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <strings.h>
#include <publisher_subscriber/pub_sub.hpp>
#include <map>
#include <sstream>

using namespace std;
using namespace ros;

// This is a special class for making the publish and subscribe possible
// The functions inside these classes are listed below:
/* Functions:
 * 1) publish_data(char *data);
 * 2) subscribe_data(void *callback_function);
*/

void no_callback_defined(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("No Callback defined for subscriber");
}

void callback_function(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("\nData Received: [%s]", msg->data.c_str());
}

Pub_Sub::Pub_Sub()
{

}

int Pub_Sub::init(int argc, char **argv, ros::NodeHandle &node_handle, msg_details &message_details)
{ 
    string utility;
    if(message_details.is_talker)
    {
        utility = "talker";
    }
    else if(message_details.is_listener)
    {
        utility = "listener";
    }

    topic = message_details.message_topic;

    cout << utility << endl;

    if(message_details.is_talker)
    {
        publish_chatter = node_handle.advertise<std_msgs::String>(topic.c_str(), 1);

        if(message_details.pub_repeat)
        {
            // Set the other two variables here since there was a repeat request.
            repeat = true;
            frequency = message_details.frequency;
        }
        else
        {
            // Set the other two variables to 0 since there was no repeat request.
            repeat = false;
            frequency = 0;
        }
    }
    else if(message_details.is_listener)
    {
        ROS_INFO("Subscribing to topic: [%s]", topic.c_str());
        node_handle.subscribe(topic.c_str(), 1, message_details.received_callback);
        ROS_INFO("Subscribed to topic: [%s]", topic.c_str());
    }
    else {
        cout << "Neither Publisher not Subscriber selected" << endl;
    }
}

int Pub_Sub::publisher_data(string data)
{
    message_stream.str("");
    message_stream << data;
    message.data = message_stream.str();
    publish_chatter.publish(message);

    // Replacement for printf
    ROS_INFO("\nPublish to topic: %s. Data: %s", topic.c_str(), message.data.c_str());

    return 0;
}

int Pub_Sub::subscribe_data(void(*received_callback)(const std_msgs::String::ConstPtr &))
{

    return 0;
}


