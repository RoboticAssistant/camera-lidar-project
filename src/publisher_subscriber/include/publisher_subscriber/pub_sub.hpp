#ifndef PUB_SUB_HPP
#define PUB_SUB_HPP

#include <iostream>
#include <std_msgs/String.h>

using namespace std;

class Pub_Sub
{

public:
    // utility: Selecting amongst "talker" or "listener"
    Pub_Sub(int argc, char **argv, string utility);

    // For publishing data
    int publisher_data(string data);

    // For subscribing data
    int subscribe_data();

private:
    ros::Publisher publish_chatter;
    ros::Subscriber subscribe_chatter;
    std_msgs::String message;
    stringstream message_stream;
};

#endif // PUB_SUB_HPP

