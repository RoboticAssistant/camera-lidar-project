#ifndef PUB_SUB_HPP
#define PUB_SUB_HPP

#include <iostream>
#include <std_msgs/String.h>

using namespace std;

// If the count variable is 0
typedef struct message_details
{
    bool is_talker;
    bool is_listener;
    string message_topic;

    // If talker
    bool pub_repeat;
    int frequency;

    // If subscriber
    void(*received_callback)(const std_msgs::String::ConstPtr &);
}msg_details;

class Pub_Sub
{

public:
    // utility: Selecting amongst "talker" or "listener"
    Pub_Sub(int argc, char **argv, msg_details &message_details);

    // For publishing data
    int publisher_data(string data);

    // For subscribing data
    int subscribe_data(void(*)(const std_msgs::String::ConstPtr &));

private:
    ros::Publisher publish_chatter;
    ros::Subscriber subscribe_chatter;
    std_msgs::String message;
    string topic;
    stringstream message_stream;
    bool repeat;
    int frequency;
};

#endif // PUB_SUB_HPP

