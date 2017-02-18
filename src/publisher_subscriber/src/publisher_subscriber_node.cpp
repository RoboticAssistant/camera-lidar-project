#include <iostream>
#include <ros/ros.h>
#include <publisher_subscriber/pub_sub.hpp>

// This project is mainly used for the publishing and subsribing of data.
// Everywhere in the project, you have to use this publisher and subscriber thing.
// There is no need to create any special pub sub models in your classes.

int main(int argc, char *argv[])
{
    Pub_Sub("Temp", "temp", "Temp");
    return 0;
}
