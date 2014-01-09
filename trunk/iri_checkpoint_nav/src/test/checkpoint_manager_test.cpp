/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include "rviz_marker_stub.cpp"

#include "../../src/checkpoint_manager.h"

TEST(CheckpointManager, check_checkpoint_srv)
{
    RvizMarkerStub rviz_stub;
    iri_checkpoint_nav::Checkpoint checkpoint;
    ros::NodeHandle n;

    /* sleep one sec help to connect to rviz if it is being used. 
     * Otherwise it won't display the checkpoints */
    sleep(5);

    // Send a service over the topic "add_nav_checkpoint"
    checkpoint.request.ref_frame = "/map";
    checkpoint.request.position.x = 1;
    checkpoint.request.position.y = 1;
    checkpoint.request.position.z = 0;

    ros::ServiceClient client = n.serviceClient<iri_checkpoint_nav::Checkpoint>
                                                                       ("add_nav_checkpoint");
    client.call(checkpoint);
    usleep(400); // some time is needed to transmit the msg through topic

    ASSERT_EQ(rviz_stub.count_markers_received(),1);
}

TEST(CheckpointManager, checkpoint_start_srv)
{
    RvizMarkerStub rviz_stub;
    iri_checkpoint_nav::ActionCheckpointNav reset_msg;
    iri_checkpoint_nav::ActionCheckpointNav start_msg;

    ros::NodeHandle n;

    reset_msg.request.action = 
        iri_checkpoint_nav::ActionCheckpointNav::Request::RESET_CHECKPOINTS;
    start_msg.request.action = 
        iri_checkpoint_nav::ActionCheckpointNav::Request::START_NAV;

    ros::ServiceClient client = n.serviceClient<iri_checkpoint_nav::ActionCheckpointNav>
                                                                      ("action_checkpoint_nav");
    client.call(reset_msg);
    usleep(200);
    ASSERT_TRUE(reset_msg.response.success);

    client.call(start_msg);
    usleep(200);
    ASSERT_TRUE(start_msg.response.success);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "checkpoint_manager_test");

    // create asynchronous thread for handling service requests
    ros::AsyncSpinner spinner(1);
    spinner.start();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
