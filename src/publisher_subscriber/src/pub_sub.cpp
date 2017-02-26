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

Pub_Sub::Pub_Sub(int argc, char **argv, string utility)
{ 
    ros::init(argc, argv, utility);
}

int Pub_Sub::publisher_data(string data, string publish_topic)
{
    ros::NodeHandle node_handle;
    publish_chatter = node_handle.advertise<std_msgs::String>(publish_topic, 1000);

    // This section has to be called just when there is need of sending continuos messages.
    // For sending just one message, this is not needed. 10 -10Hz. USE THIS PART OF CODE FROM WHERE DATA IS TRANSMITTED
    // ros::Rate loop_rate(10);
    // while(ros::ok())
    // You can write after this

    message_stream << data;
    message.data = message_stream.str();
    publish_chatter.publish(message);

    // Replacement for printf
    ROS_INFO("Publish to topic: chatter. Data: %s", message.data.c_str());

    ros::spinOnce();

    return 0;
}

void callback_function(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Data Received: [%s]", msg->data.c_str());
}

int Pub_Sub::subscribe_data(void(*received_callback)(const std_msgs::String::ConstPtr &), string subscribe_topic)
{
    ROS_INFO("Subscribed to topic: chatter");

    ros::NodeHandle node_handle;
    subscribe_chatter = node_handle.subscribe(subscribe_topic, 1000, received_callback);

    ros::spin();

    return 0;
}


