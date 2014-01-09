/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <iostream>

class RvizMarkerStub
{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber marker_sub_;

        int count_;
        visualization_msgs::Marker last_msg_;
        void marker_received_callback(const visualization_msgs::Marker & marker);

    public:
        RvizMarkerStub();

        visualization_msgs::Marker get_last_marker();
        int count_markers_received();
};

RvizMarkerStub::RvizMarkerStub() :
    count_(0)
{
    marker_sub_ = nh_.subscribe("visualization_marker", 1, & RvizMarkerStub::marker_received_callback, this);
}

void
RvizMarkerStub::marker_received_callback(const visualization_msgs::Marker & marker)
{
    count_++;
    last_msg_ = marker;
}

visualization_msgs::Marker
RvizMarkerStub::get_last_marker()
{
    return last_msg_;
}

int
RvizMarkerStub::count_markers_received()
{
    return count_;
}
