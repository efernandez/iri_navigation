/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "rviz_marker_stub.cpp"

#include "../../src/checkpoint_marker.h"

TEST(RvizMarkerStub, basic)
{
    RvizMarkerStub rviz_stub;

    ASSERT_EQ(rviz_stub.count_markers_received(), 0);
}

TEST(CheckpointMarker, check_conection)
{
    CheckpointMarker marker_server;
    RvizMarkerStub rviz_stub;
    CheckpointRequest position;

    /* sleep one sec help to connect to rviz if it is being used. 
     * Otherwise it won't display the checkpoints */
    sleep(1);

    marker_server.display_new_checkpoint(position);
    usleep(200); // some time is needed to transmit the msg through topic

    ASSERT_EQ(rviz_stub.count_markers_received(),1);

    marker_server.display_active_checkpoint(position);
    usleep(200);

    ASSERT_EQ(rviz_stub.count_markers_received(),2);

    marker_server.display_visited_checkpoint(position);
    usleep(200);

    ASSERT_EQ(rviz_stub.count_markers_received(),3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "checkpoint_marker_test");

    // create asynchronous thread for handling service requests
    ros::AsyncSpinner spinner(1);
    spinner.start();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
