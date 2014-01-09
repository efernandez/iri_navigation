#include <ros/ros.h>
#include <iri_checkpoint_nav/Checkpoint.h>
#include <iri_checkpoint_nav/ActionCheckpointNav.h>

#include <vector>
#include <iostream>
#include <fstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "checkpoint_marker_example");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    int count = 0;
    std::string ref_frame;
    std::string input_file_path;
    bool invert_x, invert_y, first_value_is_y;
    char option;

    ROS_INFO("Starting importer");

    // wait a couple of seconds to give time to manager to be available
    // when using together
    sleep(2);

    // Get the file path from the param server
    if (! pn.getParam("checkpoints_file_path", input_file_path)){
        ROS_ERROR("Node need a param called checkpoints_file_path");
        return 1;
    }

    // Change first value by y instead of x
    pn.param<bool>("first_value_is_y", first_value_is_y, false);

    // Sometimes is needed to invert X and Y values
    pn.param<bool>("invert_x", invert_x, false);
    pn.param<bool>("invert_y", invert_y, false);

    // Ref frame can be readed from param server
    pn.param<std::string>("ref_frame", ref_frame, "/map");

    std::ifstream input_file;
    input_file.open(input_file_path.c_str());

    if (! input_file.good()) {
        ROS_ERROR("Input file not found: %s", input_file_path.c_str());
        return 1;
    }

    iri_checkpoint_nav::ActionCheckpointNav reset_msg, start_msg;

    reset_msg.request.action = iri_checkpoint_nav::ActionCheckpointNav::Request::RESET_CHECKPOINTS;
    start_msg.request.action = iri_checkpoint_nav::ActionCheckpointNav::Request::START_NAV;

    ros::ServiceClient add_client    = n.serviceClient<iri_checkpoint_nav::Checkpoint>
                                                                       ("add_nav_checkpoint");

    ros::ServiceClient action_client = n.serviceClient<iri_checkpoint_nav::ActionCheckpointNav>
                                                                    ("action_checkpoint_nav");

    action_client.call(reset_msg);

    while (input_file.good()) {
        iri_checkpoint_nav::Checkpoint checkpoint;
        double x,y;
        count++;

        // Check which value is the first
        if (first_value_is_y) {
            input_file >> y;
            input_file >> x;
        } else {
            input_file >> x;
            input_file >> y;
        }

        if (invert_x)
            x = -x;

        if (invert_y)
            y = -y;

        if (! input_file.good())
            break;

        std::cout << "Add checkpoint x: " << x << " y: " << y << std::endl;

        checkpoint.request.id         = count;
        checkpoint.request.ref_frame  = ref_frame;
        checkpoint.request.position.x = x;
        checkpoint.request.position.y = y;
        checkpoint.request.position.z = 0;

        add_client.call(checkpoint);
    }

    input_file.close();
    std::cout << "Any key to start navigation (c to cancel)? ";
    std::cin >> option;

    if (option == 'c')
        return 0;

    action_client.call(start_msg);

    return 0;
}
