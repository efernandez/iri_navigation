// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 

#ifndef _checkpoint_marker_h_
#define _checkpoint_marker_h

#include <ros/ros.h>
#include <iri_checkpoint_nav/Checkpoint.h>
#include <visualization_msgs/Marker.h>
#include <iostream>

typedef iri_checkpoint_nav::Checkpoint::Request CheckpointRequest;

class CheckpointMarker
{
    private:
        visualization_msgs::Marker generate_common_marker_msg(
                                                     CheckpointRequest checkpoint);
        visualization_msgs::Marker generate_new_marker_msg(
                                                     CheckpointRequest checkpoint);
        visualization_msgs::Marker generate_visited_marker_msg(
                                                     CheckpointRequest checkpoint);
        visualization_msgs::Marker generate_active_marker_msg(
                                                     CheckpointRequest checkpoint);

        void display_checkpoint(visualization_msgs::Marker marker);

        const static std::string marker_header_frame_;
        const static std::string marker_ns_;

        ros::NodeHandle nh_;
        ros::Publisher rviz_pub;

    public:
        CheckpointMarker();

        void display_new_checkpoint(CheckpointRequest checkpoint);
        void display_visited_checkpoint(CheckpointRequest checkpoint);
        void display_active_checkpoint(CheckpointRequest checkpoint);
};

#endif
