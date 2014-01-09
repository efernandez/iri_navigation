/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include "../../src/checkpoint_marker.h"
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "checkpoint_marker_example");

    std::vector<CheckpointRequest> goals;
    std::vector<CheckpointRequest>::iterator it;
    std::vector<CheckpointRequest>::iterator prev;
    std::vector<CheckpointRequest>::iterator others;

    CheckpointMarker marker;

    ros::NodeHandle n;

    ROS_INFO("Please start rviz");
    sleep(5);

    for (int i = 0; i < 5; i++) {
        CheckpointRequest goal;
        goal.id = i;
        goal.ref_frame = "/map";
        goal.position.x = i;
        goal.position.y = i;
        goals.push_back(goal);
    }

    for (it = goals.begin(); it < goals.end(); it++) {
        marker.display_new_checkpoint(* it);
        sleep(1);
    }

    for (it = goals.begin(); it < goals.end(); it++) {
        marker.display_active_checkpoint(* it);
        sleep(1);
        if (it > goals.begin()) {
            prev = it;
            prev--;
            marker.display_visited_checkpoint(* prev);
            sleep(1);
        }
    }

    return 0;
}
