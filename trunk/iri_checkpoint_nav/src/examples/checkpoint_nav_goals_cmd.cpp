#include <ros/ros.h>
#include <iri_checkpoint_nav/Checkpoint.h>
#include <iri_checkpoint_nav/ActionCheckpointNav.h>

#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "checkpoint_marker_example");
    ros::NodeHandle n;
    int count = 0;

    iri_checkpoint_nav::ActionCheckpointNav reset_msg, start_msg;

    reset_msg.request.action = iri_checkpoint_nav::ActionCheckpointNav::Request::RESET_CHECKPOINTS;
    start_msg.request.action = iri_checkpoint_nav::ActionCheckpointNav::Request::START_NAV;

    ros::ServiceClient add_client    = n.serviceClient<iri_checkpoint_nav::Checkpoint>
                                                                       ("add_nav_checkpoint");

    ros::ServiceClient action_client = n.serviceClient<iri_checkpoint_nav::ActionCheckpointNav>
                                                                    ("action_checkpoint_nav");

    action_client.call(reset_msg);

    while (1) {
        iri_checkpoint_nav::Checkpoint checkpoint;
        char option;
        count++;

        checkpoint.request.id = count;
        checkpoint.request.ref_frame = "/map";

        std::cout << "New navigation checkpoint" << std::endl << std::endl;
        std::cout << "Position X: ";
        std::cin >> checkpoint.request.position.x;
        std::cout << std::endl << "Position Y: ";
        std::cin >> checkpoint.request.position.y;
        std::cout << std::endl;

        add_client.call(checkpoint);
        std::cout << "Another checkpoint (Y/n)? ";
        std::cin >> option;

        if (option == 'n')
            break;
    }

    std::cout << "Starting navigation:";
    action_client.call(start_msg);

    return 0;
}
