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

int process_data(bool process_direction, bool process_rotation) {
    static string forward_data("forward");
    static string backward_data("backward");
    static string stop_data("stop");
    static string left_data("left");
    static string right_data("right");

    if(process_direction) {
        // Checking for forward or backward motion
        if(!linear_displacement_data.compare("FORWARD")) {
            d_m_direction_publisher.publisher_data(forward_data);
        }
        else if(!linear_displacement_data.compare("BACKWARD")){
            d_m_direction_publisher.publisher_data(backward_data);
        }
        else if(!linear_displacement_data.compare("STOP")){
            d_m_direction_publisher.publisher_data(stop_data);
        }
        else {
            d_m_direction_publisher.publisher_data(stop_data);
        }
    }

    if(process_rotation) {
        // Checking for direction of motion: Left or Right
        if(!angular_displacement_data.compare("LEFT")) {
            d_m_rotation_publisher.publisher_data(left_data);
        }
        else if(!angular_displacement_data.compare("RIGHT")) {
            d_m_rotation_publisher.publisher_data(right_data);
        }
        else if(!angular_displacement_data.compare("STOP")) {
            d_m_rotation_publisher.publisher_data(stop_data);
        }
    }

    return 0;
}
