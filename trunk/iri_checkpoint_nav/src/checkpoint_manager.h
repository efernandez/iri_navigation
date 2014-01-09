/* vim: set sw=4 sts=4 et foldmethod=syntax : */

#ifndef SRC_GUARD_CHECKPOINT_MANAGER_H
#define SRC_GUARD_CHECKPOINT_MANAGER_H 1

#include <ros/ros.h>
#include "checkpoint_marker.h"
#include <iri_checkpoint_nav/Checkpoint.h>
#include <iri_checkpoint_nav/ActionCheckpointNav.h>
#include <tf/transform_broadcaster.h>

#include <vector>


class CheckpointManager
{
    private:
        ros::ServiceServer srv_checkpoint_;
        ros::ServiceServer srv_action_;
        ros::NodeHandle nh_;

        CheckpointMarker marker_;
        std::vector<CheckpointRequest> checkpoints_;
        std::vector<CheckpointRequest>::iterator current_goal_;

        bool add_checkpoint_callback(iri_checkpoint_nav::Checkpoint::Request  & req,
                                     iri_checkpoint_nav::Checkpoint::Response & res);
        bool action_callback(iri_checkpoint_nav::ActionCheckpointNav::Request  & req,
                                     iri_checkpoint_nav::ActionCheckpointNav::Response & res);

        /*
         * Orientation is always the direction from current goal to the next
         * goal. The only exception is in the last goal, when direction is get
         * from previous goal to current
         */
        geometry_msgs::Quaternion generate_goal_orientation();
        static geometry_msgs::Quaternion generate_default_orientation();

        void activate_current_checkpoint();
        void visited_current_checkpoint();
        void reset_checkpoints();
        bool send_navigation_goal(const CheckpointRequest checkpoint);
        bool start_navigation();

    public:
        CheckpointManager();
};

#endif
