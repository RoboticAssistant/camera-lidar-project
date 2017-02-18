#include <iostream>
#include <ros/ros.h>
#include <publisher_subscriber/pub_sub.hpp>

// This project is mainly used for the publishing and subsribing of data.
// Everywhere in the project, you have to use this publisher and subscriber thing.
// There is no need to create any special pub sub models in your classes.

int main(int argc, char *argv[])
{
    if(!strcmp(argv[1], "talker"))
    {
        string send_data(argv[2]);
        cout << "Talker" << endl;
        Pub_Sub publisher(argc, argv, "talker");

        while(1)
        {
            // Publishing data
            publisher.publisher_data(send_data);
        }
    }
    else if(!strcmp(argv[1], "listener"))
    {
        cout << "Listener" << endl;
        Pub_Sub subscriber(argc, argv, "listener");
        subscriber.subscribe_data();
    }

    return 0;
}
