#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <publisher_subscriber/pub_sub.hpp>
#include <string.h>
#include <decision_module/decision_processing.hpp>

using namespace std;

// This file deals with the reception and algorithm for the
// motor movement

extern string linear_displacement_data;
extern string angular_displacement_data;

extern Pub_Sub d_m_direction_publisher;
extern Pub_Sub d_m_rotation_publisher;

int process_data() {
    static string forward_data("forward");
    static string backward_data("backward");
    static string stop_data("stop");
    static string left_data("left");
    static string right_data("right");
    static string data_not_set("NO_USEFUL_DATA");

    // Checking for forward or backward motion
    if(!linear_displacement_data.compare("forward")) {
        d_m_direction_publisher.publisher_data(forward_data);
    }
    else if(!linear_displacement_data.compare("backward")){
        d_m_direction_publisher.publisher_data(backward_data);
    }
    else if(!linear_displacement_data.compare("stop")){
        d_m_direction_publisher.publisher_data(stop_data);
    }
    else {
        d_m_direction_publisher.publisher_data(data_not_set);
    }

    // Checking for direction of motion: Left or Right
    if(!angular_displacement_data.compare("left")) {
        d_m_rotation_publisher.publisher_data(left_data);
    }
    else if(!angular_displacement_data.compare("right")){
        d_m_rotation_publisher.publisher_data(right_data);
    }
    else {
//        d_m_rotation_publisher.publisher_data(data_not_set);
    }

    return 0;
}
