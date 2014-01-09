#include "checkpoint_manager.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

CheckpointManager::CheckpointManager()
{
    srv_checkpoint_ = nh_.advertiseService("add_nav_checkpoint",
                                           & CheckpointManager::add_checkpoint_callback, this);

    srv_action_ = nh_.advertiseService("action_checkpoint_nav",
                                           & CheckpointManager::action_callback, this);
}

bool
CheckpointManager::add_checkpoint_callback(iri_checkpoint_nav::Checkpoint::Request  & req,
                                           iri_checkpoint_nav::Checkpoint::Response & res)
{
    // Autogenerate the ID each time a new checkpoint is recieved
    req.id = checkpoints_.size();

    checkpoints_.push_back(req);
    marker_.display_new_checkpoint(req);

    return true;
}

bool
CheckpointManager::action_callback(iri_checkpoint_nav::ActionCheckpointNav::Request  & req,
                                   iri_checkpoint_nav::ActionCheckpointNav::Response & res)
{
    switch (req.action)
    {
        case iri_checkpoint_nav::ActionCheckpointNav::Request::START_NAV:
            ROS_INFO("Recieved start navigation action");
            res.success = start_navigation();
            break;
        case iri_checkpoint_nav::ActionCheckpointNav::Request::RESET_CHECKPOINTS:
            ROS_INFO("Recieved reset action");
            reset_checkpoints();
            res.success = true;
            break;
        default:
            res.success = false;
    }

    return res.success;
}

void
CheckpointManager::activate_current_checkpoint()
{
    marker_.display_active_checkpoint(* current_goal_);
}

void
CheckpointManager::visited_current_checkpoint()
{
    marker_.display_visited_checkpoint(* current_goal_);
}

void
CheckpointManager::reset_checkpoints()
{
    checkpoints_.clear();
}

geometry_msgs::Quaternion
CheckpointManager::generate_default_orientation()
{
    geometry_msgs::Quaternion quat;

    quat.x = 0;
    quat.y = 0;
    quat.z = 0;
    quat.w = 1;

    return quat;
}

geometry_msgs::Quaternion
CheckpointManager::generate_goal_orientation()
{
    // Check if we have only one goal
    if ((current_goal_ == --checkpoints_.end()) && (current_goal_ == checkpoints_.begin()))
        return generate_default_orientation();

    std::vector<CheckpointRequest>::iterator ref_goal = current_goal_;
    CheckpointRequest start;
    CheckpointRequest end;

    if (current_goal_ == --checkpoints_.end()) {
        // Last checkpoint. Orientation is from previous to current.
        ref_goal--;
        start = * ref_goal;
        end   = * current_goal_;
    } else {
        // Not in the last checkpoint. Orientation is from current to next.
        ref_goal++;
        start = * current_goal_;
        end   = * ref_goal;
    }

    // ArcTan without singularities (atan2) will give the angle in radians
    double rotation_angle = atan2(end.position.y - start.position.y,
                                  end.position.x - start.position.x);

    return tf::createQuaternionMsgFromYaw(rotation_angle);
}

bool
CheckpointManager::send_navigation_goal(const CheckpointRequest checkpoint)
{
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (! ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id    = checkpoint.ref_frame;
    goal.target_pose.header.stamp       = ros::Time::now();
    goal.target_pose.pose.position      = checkpoint.position;
    goal.target_pose.pose.orientation   = generate_goal_orientation();

    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_WARN("The base failed to move");
        return false;
    }

    return true;
}

bool
CheckpointManager::start_navigation()
{
    std::vector<CheckpointRequest>::iterator it;

    for (it = checkpoints_.begin(); it < checkpoints_.end(); it++) {
        ROS_INFO("Sent navigation goal");
        current_goal_ = it;
        // Display navigation point in rviz
        activate_current_checkpoint();
        // Send goal to the navigation stack and wait
        if (! send_navigation_goal(* it))
            ROS_WARN("Navigation to the goal failed. Send to the next goal");

        visited_current_checkpoint();
    }

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "checkpoint_manager");
    CheckpointManager server;

    ros::spin();

    return 0;
}
