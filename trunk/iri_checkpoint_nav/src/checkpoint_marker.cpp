#include "checkpoint_marker.h"

const std::string CheckpointMarker::marker_header_frame_ = "/map";
const std::string CheckpointMarker::marker_ns_ = "";

CheckpointMarker::CheckpointMarker()
{
    rviz_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
}

visualization_msgs::Marker
CheckpointMarker::generate_common_marker_msg(CheckpointRequest checkpoint)
{
    visualization_msgs::Marker marker;

    if (checkpoint.ref_frame == "")
        ROS_WARN("Checkpoint without reference frame. It will not display anything");

    marker.header.frame_id = checkpoint.ref_frame;
    marker.header.stamp    = ros::Time();
    marker.ns              = marker_ns_;
    marker.id              = checkpoint.id;
    marker.type            = visualization_msgs::Marker::SPHERE;
    marker.action          = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.pose.position = checkpoint.position;

    return marker;
}


visualization_msgs::Marker
CheckpointMarker::generate_new_marker_msg(CheckpointRequest checkpoint)
{
    visualization_msgs::Marker marker = generate_common_marker_msg(checkpoint);

    // yellow (255,255,0, 60%)
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.6;

    return marker;
}

visualization_msgs::Marker
CheckpointMarker::generate_visited_marker_msg(CheckpointRequest checkpoint)
{
    visualization_msgs::Marker marker = generate_common_marker_msg(checkpoint);

    // grey (128,128,128,80%)
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 0.8;

    return marker;
}

visualization_msgs::Marker
CheckpointMarker::generate_active_marker_msg(CheckpointRequest checkpoint)
{
    visualization_msgs::Marker marker = generate_common_marker_msg(checkpoint);

    // red (255,0,0,90%)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.9;

    return marker;
}

void
CheckpointMarker::display_checkpoint(visualization_msgs::Marker marker)
{
    rviz_pub.publish(marker);
}

void
CheckpointMarker::display_new_checkpoint(CheckpointRequest checkpoint)
{
    display_checkpoint(generate_new_marker_msg(checkpoint));
}

void
CheckpointMarker::display_visited_checkpoint(CheckpointRequest checkpoint)
{
    display_checkpoint(generate_visited_marker_msg(checkpoint));
}

void
CheckpointMarker::display_active_checkpoint(CheckpointRequest checkpoint)
{
    display_checkpoint(generate_active_marker_msg(checkpoint));
}
