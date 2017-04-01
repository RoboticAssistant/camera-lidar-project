#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <publisher_subscriber/pub_sub.hpp>
#include <string.h>
#include <decision_module/decision_processing.hpp>

using namespace std;
//#define TEST_CODE

string linear_displacement_data;
string angular_displacement_data;

Pub_Sub a_d_direction_subscriber;
Pub_Sub a_d_rotation_subscriber;
Pub_Sub d_m_direction_publisher;
Pub_Sub d_m_rotation_publisher;

/// This function is a callback for following details
// 1) Audio to Motor: For forward backward direction
void a_d_direction_callback(const std_msgs::String::ConstPtr &msg)
{
    // Activate logging
    ROS_INFO("decision_module: a_d_direction_callback(): Data Received: [%s]", msg->data.c_str());
    linear_displacement_data = msg->data;
}

/// This function is a callback for following details
// 1) Audio to Motor: For rotation of motor direction
void a_d_rotation_callback(const std_msgs::String::ConstPtr &msg)
{
    // Activate logging
    ROS_INFO("decision_module: a_d_rotation_callback(): Data Received: [%s]", msg->data.c_str());
    angular_displacement_data = msg->data;
}


// There is no need for any command line arguments here in this module
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "decision_module");
    ros::NodeHandle node_handle;
//    ros::NodeHandle node_handle;

#ifdef TEST_CODE
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
    }
#else
    // Define subscriber from audio to decision
    // Subscriber 1: From Audio to Decision: Forward and Backward data
    string topic;
    msg_details a_d_direction_details;
    a_d_direction_details.is_talker = false;
    a_d_direction_details.is_listener = true;
    a_d_direction_details.pub_repeat = false;
    a_d_direction_details.frequency = 1;
    a_d_direction_details.message_topic = "A_D_direction";
    a_d_direction_details.received_callback = a_d_direction_callback;
    topic = a_d_direction_details.message_topic;
    ros::Subscriber sub_1 = node_handle.subscribe(topic, 1000, a_d_direction_callback);
//    a_d_direction_subscriber.init(argc, argv, node_handle, a_d_direction_details);

    // Subscriber 2: From Audio to Decision: Rotation left or right
    msg_details a_d_rotation_details;
    a_d_rotation_details.is_talker = false;
    a_d_rotation_details.is_listener = true;
    a_d_rotation_details.pub_repeat = false;
    a_d_rotation_details.frequency = 1;
    a_d_rotation_details.message_topic = "A_D_rotation";
    a_d_rotation_details.received_callback = a_d_rotation_callback;
    topic = a_d_rotation_details.message_topic;
    ros::Subscriber sub_2 = node_handle.subscribe(topic, 1000, a_d_rotation_callback);
//    a_d_rotation_subscriber.init(argc, argv, node_handle_sub, a_d_rotation_details);

    // Publisher 1: From Decision to Motor: Forward
    msg_details d_m_direction_details;
    d_m_direction_details.is_talker = true;
    d_m_direction_details.is_listener = false;
    d_m_direction_details.pub_repeat = false;
    d_m_direction_details.frequency = 1;
    d_m_direction_details.message_topic = "D_M_direction";
    d_m_direction_publisher.init(argc, argv, node_handle, d_m_direction_details);

    // Publisher 2: From Decision to Motor: Rotation left or right
    msg_details d_m_rotation_details;
    d_m_rotation_details.is_talker = true;
    d_m_rotation_details.is_listener = false;
    d_m_rotation_details.pub_repeat = false;
    d_m_rotation_details.frequency = 1;
    d_m_rotation_details.message_topic = "D_M_rotation";
    d_m_rotation_publisher.init(argc, argv, node_handle, d_m_rotation_details);
#endif

    ros::Rate loop_rate(10);
    // While loop for continuously assessing the system and running
    while(ros::ok()) {
        process_data();

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
