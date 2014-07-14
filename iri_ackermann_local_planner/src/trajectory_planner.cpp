/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <trajectory_planner.h>

using namespace std;
using namespace costmap_2d;

namespace iri_ackermann_local_planner{
  void TrajectoryPlanner::reconfigure(AckermannLocalPlannerConfig &cfg) 
  {
    iri_ackermann_local_planner::AckermannLocalPlannerConfig config(cfg);
    boost::mutex::scoped_lock l(configuration_mutex_);

    sim_time_ = config.sim_time;
    sim_granularity_ = config.sim_granularity;

    pdist_scale_ = config.pdist_scale;
    gdist_scale_ = config.gdist_scale;
    hdiff_scale_ = config.hdiff_scale;
    occdist_scale_ = config.occdist_scale;

    vx_samples_ = config.vx_samples;
    vtheta_samples_ = config.vtheta_samples;

    if (vx_samples_ <= 0) 
    {
      config.vx_samples = 1;
      vx_samples_ = config.vx_samples;
      ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
    }
    if(vtheta_samples_ <= 0) 
    {
      config.vtheta_samples = 1;
      vtheta_samples_ = config.vtheta_samples;
      ROS_WARN("You've specified that you don't want any samples in the theta dimension. We'll at least assume that you want to sample one value... so we're going to set vtheta_samples to 1 instead");
    }

    simple_attractor_ = config.simple_attractor;

    angular_sim_granularity_ = config.angular_sim_granularity;

    /* ackerman parameters */
    this->ack_vel_max_=config.ack_vel_max;//config.ack_vel_max;
    this->ack_vel_min_=config.ack_vel_max;//config.ack_vel_min;
    this->ack_acc_max_=config.ack_acc_max;//config.ack_acc_max;
    this->ack_steer_angle_max_=config.ack_steer_angle_max;//config.ack_steer_angle_max;
    this->ack_steer_angle_min_=config.ack_steer_angle_min;//config.ack_steer_angle_min;
    this->ack_steer_speed_max_=config.ack_steer_speed_max;//config.ack_steer_speed_max;
    this->ack_steer_speed_min_=config.ack_steer_speed_min;//config.ack_steer_speed_min;
    this->ack_steer_acc_max_=config.ack_steer_acc_max;//config.ack_steer_acc_max;
    this->ack_axis_distance_=config.ack_axis_distance;
    this->heading_points_=config.heading_points;
  }

  TrajectoryPlanner::TrajectoryPlanner(WorldModel &world_model,
    const costmap_2d::Costmap2D& costmap,
    std::vector<geometry_msgs::Point> footprint_spec,
    double max_acc, double max_vel, double min_vel,
    double max_steer_acc, double max_steer_vel, double min_steer_vel,
    double max_steer_angle, double min_steer_angle,double axis_distance,
    double sim_time, double sim_granularity,
    int vx_samples, int vtheta_samples,
    double pdist_scale, double gdist_scale, double occdist_scale, double hdiff_scale,
    bool simple_attractor, double angular_sim_granularity,int heading_points,double xy_goal_tol)
    : map_(costmap.getSizeInCellsX(), costmap.getSizeInCellsY()), costmap_(costmap), 
    world_model_(world_model), footprint_spec_(footprint_spec),
    sim_time_(sim_time), sim_granularity_(sim_granularity), angular_sim_granularity_(angular_sim_granularity),
    vx_samples_(vx_samples), vtheta_samples_(vtheta_samples),
    pdist_scale_(pdist_scale), gdist_scale_(gdist_scale), occdist_scale_(occdist_scale),hdiff_scale_(hdiff_scale),
    ack_acc_max_(max_acc), ack_vel_min_(min_vel), ack_vel_max_(max_vel),
    ack_steer_acc_max_(max_steer_acc),ack_steer_speed_max_(max_steer_vel),ack_steer_speed_min_(min_steer_vel),
    ack_steer_angle_max_(max_steer_angle),ack_steer_angle_min_(min_steer_angle),ack_axis_distance_(axis_distance),
    simple_attractor_(simple_attractor),heading_points_(heading_points),xy_goal_tol_(xy_goal_tol)
  {
  }

  TrajectoryPlanner::~TrajectoryPlanner(){}

  bool TrajectoryPlanner::getCellCosts(int cx, int cy, float &path_cost, float &goal_cost, float &occ_cost, float &total_cost) {
    MapCell cell = map_(cx, cy);
    if (cell.within_robot) {
        return false;
    }
    occ_cost = costmap_.getCost(cx, cy);
    if (cell.path_dist >= map_.map_.size() || cell.goal_dist >= map_.map_.size() || occ_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
    }
    path_cost = cell.path_dist;
    goal_cost = cell.goal_dist;
    total_cost = pdist_scale_ * path_cost + gdist_scale_ * goal_cost + occdist_scale_ * occ_cost;
    return true;
  }

  //create and score a trajectory given the current pose of the robot and selected velocities
  void TrajectoryPlanner::generateTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp, 
      double acc_x, double acc_y, double acc_theta, double impossible_cost,
      Trajectory& traj){

    // make sure the configuration doesn't change mid run
    boost::mutex::scoped_lock l(configuration_mutex_);

    // store the current state of the robot (pose (x,y,theta) and speed
    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    // ackerman current state
    double vt_i;
    double steer_angle_i,steer_speed_i;
    double vx_int,vy_int;
    double r,d;

    // for the ackerman configuration, vx_samp -> v_t and vtheta_samp -> steer_angle
    vt_i=vx;
    steer_angle_i=vy;
    steer_speed_i=vtheta;

    vx_int = vx_samp;
    vy_int = 0;

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    num_steps = int(sim_time_ / sim_granularity_ + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if(num_steps == 0)
      num_steps = 1;

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_int; 
    traj.yv_ = vy_int; 
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;
    double heading_diff = 0.0;

    double speed=0.0,angle=0.0;
    double T1=0.0,T4=0.0,T2=0.0,T3=0.0;

    /* check wether the trajectory can be generated or not */
    if(vtheta<0 && (-vtheta*vtheta/(2*this->ack_steer_acc_max_)+vy)>this->ack_steer_angle_max_)
      return;

    if(vtheta>0 && (vtheta*vtheta/(2*this->ack_steer_acc_max_)+vy)>this->ack_steer_angle_max_)
      return;

    if(vtheta<0 && (-vtheta*vtheta/(2*this->ack_steer_acc_max_)+vy)<this->ack_steer_angle_min_)
      return;

    if(vtheta>0 && (vtheta*vtheta/(2*this->ack_steer_acc_max_)+vy)<this->ack_steer_angle_min_)
      return;

    // compute the trajectory times
    if(steer_speed_i>=0)
    {
      if(vtheta_samp>steer_angle_i)
      { 
        speed=(steer_speed_i+this->ack_steer_acc_max_*sim_time_)/2.0;
        if(speed>=this->ack_steer_speed_max_)
        {
          speed=this->ack_steer_speed_max_;  
          T4=sim_time_-(2.0*speed-steer_speed_i)/this->ack_steer_acc_max_;
          angle=speed*speed/this->ack_steer_acc_max_-steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)+speed*sim_time_*T4+steer_angle_i;
          if(angle>vtheta_samp)
          {
            angle=vtheta_samp;
            T4=(vtheta_samp-steer_angle_i-speed*speed/this->ack_steer_acc_max_+steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_))/speed;
            if(T4<0)
            {
              T4=0;
              speed=sqrt(steer_speed_i*steer_speed_i/2.0+this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i));
            }
          }
        }
        else
        { 
          angle=speed*speed/this->ack_steer_acc_max_-steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)+steer_angle_i;
          T4=0;
          if(angle>vtheta_samp)
          {
            angle=vtheta_samp;
            speed=sqrt(steer_speed_i*steer_speed_i/2.0+this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i));
          }
        }
        T1=(speed-steer_speed_i)/this->ack_steer_acc_max_;
        if(T1<0)
          return;
      }
      else
      {
        speed=(steer_speed_i-this->ack_steer_acc_max_*sim_time_)/2.0;
        if(speed<=this->ack_steer_speed_min_)
        {
          speed=this->ack_steer_speed_min_;
          T4=sim_time_+(2.0*speed-steer_speed_i)/this->ack_steer_acc_max_;
          angle=steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)-speed*speed/this->ack_steer_acc_max_+speed*T4+steer_angle_i;
          if(angle<vtheta_samp)
          {
            angle=vtheta_samp;
            T4=(vtheta_samp-steer_angle_i+speed*speed/this->ack_steer_acc_max_-steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_))/speed;
            if(T4<0)
            {
              T4=0;
              speed=-sqrt(steer_speed_i*steer_speed_i/2.0-this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i));
            }
          }
        }
        else
        {
          angle=steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)-speed*speed/this->ack_steer_acc_max_+steer_angle_i;
          T4=0;
          if(angle<vtheta_samp)
          {
            angle=vtheta_samp;
            speed=-sqrt(steer_speed_i*steer_speed_i/2.0-this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i)); 
          }
        }
        T1=-(speed-steer_speed_i)/this->ack_steer_acc_max_;
        if(T1<0)
          return;
      }
    }
    else
    {
      if(vtheta_samp>steer_angle_i)
      {
        speed=(steer_speed_i+this->ack_steer_acc_max_*sim_time_)/2.0;
        if(speed>=this->ack_steer_speed_max_)
        {
          speed=this->ack_steer_speed_max_;
          T4=sim_time_-(2.0*speed-steer_speed_i)/this->ack_steer_acc_max_;
          angle=-steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)+speed*speed/this->ack_steer_acc_max_+speed*T4+steer_angle_i;
          if(angle>vtheta_samp)
          {
            angle=vtheta_samp;
            T4=(vtheta_samp-steer_angle_i-speed*speed/this->ack_steer_acc_max_+steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_))/speed;
            if(T4<0)
            {
              T4=0;
              speed=sqrt(steer_speed_i*steer_speed_i/2.0+this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i));
            }
          }
        }
        else
        {
          angle=-steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)+speed*speed/this->ack_steer_acc_max_+steer_angle_i;
          T4=0;
          if(angle>vtheta_samp)
          {
            angle=vtheta_samp;
            speed=sqrt(steer_speed_i*steer_speed_i/2.0+this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i));
          }
        }
        T1=(speed-steer_speed_i)/this->ack_steer_acc_max_;
        if(T1<0)
          return;
      }
      else
      {
        speed=(steer_speed_i-this->ack_steer_acc_max_*sim_time_)/2.0;
        if(speed<=this->ack_steer_speed_min_)
        {
          speed=this->ack_steer_speed_min_;
          T4=sim_time_+(2.0*speed-steer_speed_i)/this->ack_steer_acc_max_;
          angle=-speed*speed/this->ack_steer_acc_max_+steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)+speed*T4+steer_angle_i;
          if(angle<vtheta_samp)
          {
            angle=vtheta_samp;
            T4=(vtheta_samp-steer_angle_i+speed*speed/this->ack_steer_acc_max_-steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_))/speed;
            if(T4<0)
            {
              T4=0;
              speed=-sqrt(steer_speed_i*steer_speed_i/2.0-this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i));
            }
          }
        }
        else
        {
          angle=-speed*speed/this->ack_steer_acc_max_+steer_speed_i*steer_speed_i/(2.0*this->ack_steer_acc_max_)+steer_angle_i;
          T4=0;
          if(angle<vtheta_samp)
          {
            angle=vtheta_samp;
            speed=-sqrt(steer_speed_i*steer_speed_i/2.0-this->ack_steer_acc_max_*(vtheta_samp-steer_angle_i)); 
          }
        }
        T1=-(speed-steer_speed_i)/this->ack_steer_acc_max_;
        if(T1<0)
          return;
      }
    }

    double v=0.0;
    if(vx_samp>vt_i)
    {
      v=(vt_i+this->ack_acc_max_*sim_time_)/2.0;
      if(v>vx_samp)
      {
        v=vx_samp;
        T3=sim_time_-(2.0*v-vt_i)/this->ack_acc_max_;
      }
      else
        T3=0;
      T2=(v-vt_i)/this->ack_acc_max_;
      if(T2<0)
        return;
    }
    else
    {
      v=(vt_i-this->ack_acc_max_*sim_time_)/2.0;
      if(v<vx_samp)
      {
        v=vx_samp;
        T3=sim_time_+(2.0*v-vt_i)/this->ack_acc_max_;
      }
      else
        T3=0;
      T2=-(v-vt_i)/this->ack_acc_max_;
      if(T2<0)
        return;
    }

    double time_window_start=sim_time_-2*dt;///heading_points_;
    double time_window_end=sim_time_-dt;///heading_points_+dt;

    for(int i = 0; i < num_steps; ++i){
      heading_diff=3.14159;
      //get map coordinates of a point
      unsigned int cell_x, cell_y;

      //we don't want a path that goes off the know map
      if(!costmap_.worldToMap(x_i, y_i, cell_x, cell_y)){
        traj.cost_ = -1.0;
        ROS_DEBUG("TrajectoryPlanner::generateTrajectory: Current point out of map!");
        return;
      }

      //check the point on the trajectory for legality
      double footprint_cost = footprintCost(x_i, y_i, theta_i);

      occ_cost = std::max(std::max(occ_cost, footprint_cost), double(costmap_.getCost(cell_x, cell_y)));

      double cell_pdist = map_(cell_x, cell_y).path_dist;
      double cell_gdist = map_(cell_x, cell_y).goal_dist;
     
      double near_dist=DBL_MAX,dist;
      unsigned int near_index=0;
 
      //update path and goal distances
      path_dist = cell_pdist;
      goal_dist = cell_gdist;
      if(time >= time_window_start && time < time_window_end)
      {
        // find the nearrest point on the path
        for(unsigned int i = 0;i<global_plan_.size();i++)
        {
          dist=sqrt((global_plan_[i].pose.position.x-x_i)*(global_plan_[i].pose.position.x-x_i)+(global_plan_[i].pose.position.y-y_i)*(global_plan_[i].pose.position.y-y_i));
          if(dist<near_dist)
          { 
            near_dist=dist;
            near_index=i;
          }
        }
        double v1_x,v1_y;
        if(near_index==0)
        {
          v1_x = global_plan_[near_index+1].pose.position.x - global_plan_[near_index].pose.position.x;
          v1_y = global_plan_[near_index+1].pose.position.y - global_plan_[near_index].pose.position.y;
        }
        else
        {
          v1_x = global_plan_[near_index].pose.position.x - global_plan_[near_index-1].pose.position.x;
          v1_y = global_plan_[near_index].pose.position.y - global_plan_[near_index-1].pose.position.y;
        }
        double v2_x = cos(theta_i);
        double v2_y = sin(theta_i);

        double perp_dot = v1_x * v2_y - v1_y * v2_x;
        double dot = v1_x * v2_x + v1_y * v2_y;

        //get the signed angle
        heading_diff = fabs(atan2(perp_dot, dot));
        if(heading_diff>(3.14159/2.0))
          heading_diff=fabs(heading_diff-3.14159);
        ROS_INFO("TrajectoryPlanner::createTrajectories: heading_dist: %f", heading_diff);
//        time_window_start+=sim_time_/heading_points_;
//        time_window_end=time_window_start+dt;
      }

      //do we want to follow blindly
      if(simple_attractor_){
        goal_dist = (x_i - global_plan_[global_plan_.size() -1].pose.position.x) * 
          (x_i - global_plan_[global_plan_.size() -1].pose.position.x) + 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y) * 
          (y_i - global_plan_[global_plan_.size() -1].pose.position.y);
        path_dist = 0.0;
      }
      else{
        //if a point on this trajectory has no clear path to goal it is invalid
        if(impossible_cost <= goal_dist || impossible_cost <= path_dist){
          ROS_DEBUG("No path to goal with goal distance = %f, path_distance = %f and max cost = %f", 
              cell_gdist, cell_pdist, impossible_cost);
          traj.cost_ = -2.0;
          return;
        }
      }


      //the point is legal... add it to the trajectory
      traj.addPoint(x_i, y_i, theta_i);

      // compute the next point in the trajectory
      if(vtheta_samp>vtheta)
      {
        if(time<T1)
        {
          steer_speed_i=steer_speed_i+this->ack_steer_acc_max_*dt;
          if(steer_speed_i>speed)
            steer_speed_i=speed;
        }
        else if(time<T1+T4)
          steer_speed_i=speed;
        else
        { 
          steer_speed_i=steer_speed_i-this->ack_steer_acc_max_*dt;
          if(steer_speed_i<0)
            steer_speed_i=0;
        }
      }
      else
      {
        if(time<T1)
        {
          steer_speed_i=steer_speed_i-this->ack_steer_acc_max_*dt;
          if(steer_speed_i<speed)
            steer_speed_i=speed;
        }
        else if(time<T1+T4)
          steer_speed_i=speed;
        else
        {
          steer_speed_i=steer_speed_i+this->ack_steer_acc_max_*dt;
          if(steer_speed_i>0)
            steer_speed_i=0;
        }
      }
      steer_angle_i+=steer_speed_i*dt;
      if(vx_samp>vx)
      {
        if(time<T2)
        {
          vt_i=vt_i+this->ack_acc_max_*dt;
          if(vt_i>v)
            vt_i=v;
        }
        else if(time<(T2+T3))
          vt_i=v;
        else
        {
          vt_i=vt_i-this->ack_acc_max_*dt;
          if(vt_i<0)
            vt_i=0;
        }
      }
      else
      {
        if(time<T2)
        {
          vt_i=vt_i-this->ack_acc_max_*dt;
          if(vt_i<v)
            vt_i=v;
        }
        else if(time<T2+T3)
          vt_i=v;
        else
        {  
          vt_i=vt_i+this->ack_acc_max_*dt;
          if(vt_i>0)
            vt_i=0;
        }
      }

      if(fabs(steer_angle_i)>0.02)
      {
        r=ack_axis_distance_*tan(3.14159/2.0-steer_angle_i);
        d=vt_i*dt;
        x_i+=d*cos(theta_i);
        y_i+=d*sin(theta_i);
        theta_i+=d/r;
      }
      else
      {
        d=vt_i*dt;
        x_i+=d*cos(theta_i);
        y_i+=d*sin(theta_i);
      }

      //increment time
      time += dt;
    }

    //ROS_INFO("OccCost: %f, vx: %.2f, vy: %.2f, vtheta: %.2f", occ_cost, vx_samp, vy_samp, vtheta_samp);
    double cost = -1.0;
    cost = (pdist_scale_ * path_dist + goal_dist * gdist_scale_ + occdist_scale_ * occ_cost + hdiff_scale_ * heading_diff)*(1.0+0.0*fabs(vy-vtheta_samp));      

    traj.cost_ = cost;
  }

  double TrajectoryPlanner::headingDiff(int cell_x, int cell_y, double x, double y, double heading){
    double heading_diff = DBL_MAX;
    unsigned int goal_cell_x, goal_cell_y;
    //find a clear line of sight from the robot's cell to a point on the path
    for(int i = global_plan_.size() - 1; i >=0; --i){
      if(costmap_.worldToMap(global_plan_[i].pose.position.x, global_plan_[i].pose.position.y, goal_cell_x, goal_cell_y)){
        if(lineCost(cell_x, goal_cell_x, cell_y, goal_cell_y) >= 0){
          double gx, gy;
          costmap_.mapToWorld(goal_cell_x, goal_cell_y, gx, gy);
          double v1_x = gx - x;
          double v1_y = gy - y;
          double v2_x = cos(heading);
          double v2_y = sin(heading);

          double perp_dot = v1_x * v2_y - v1_y * v2_x;
          double dot = v1_x * v2_x + v1_y * v2_y;

          //get the signed angle
          double vector_angle = atan2(perp_dot, dot);

          heading_diff = fabs(vector_angle);
          return heading_diff;

        }
      }
    }
    return heading_diff;
  }

  //calculate the cost of a ray-traced line
  double TrajectoryPlanner::lineCost(int x0, int x1, 
      int y0, int y1){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    double line_cost = 0.0;
    double point_cost = -1.0;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      point_cost = pointCost(x, y); //Score the current point

      if(point_cost < 0)
        return -1;

      if(line_cost < point_cost)
        line_cost = point_cost;

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }

    return line_cost;
  }

  double TrajectoryPlanner::pointCost(int x, int y){
    unsigned char cost = costmap_.getCost(x, y);
    //if the cell is in an obstacle the path is invalid
    if(cost == LETHAL_OBSTACLE || cost == INSCRIBED_INFLATED_OBSTACLE || cost == NO_INFORMATION){
      return -1;
    }

    return cost;
  }

  void TrajectoryPlanner::updatePlan(const vector<geometry_msgs::PoseStamped>& new_plan, bool compute_dists)
  {
    global_plan_.resize(new_plan.size());
    for(unsigned int i = 0; i < new_plan.size(); ++i)
    {
      global_plan_[i] = new_plan[i];
    }
    if(compute_dists){
      //reset the map for new operations
      map_.resetPathDist();
      //make sure that we update our path based on the global plan and compute costs
      map_.setPathCells(costmap_, global_plan_);
      ROS_DEBUG("Path/Goal distance computed");
    }
  }

  bool TrajectoryPlanner::checkTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t; 

    double cost = scoreTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp);

    //if the trajectory is a legal one... the check passes
    if(cost >= 0)
      return true;

    //otherwise the check fails
    return false;
  }

  double TrajectoryPlanner::scoreTrajectory(double x, double y, double theta, double vx, double vy, 
      double vtheta, double vx_samp, double vy_samp, double vtheta_samp){
    Trajectory t; 
    double impossible_cost = map_.map_.size();
    generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
        ack_acc_max_, ack_vel_max_, ack_vel_min_, impossible_cost, t);

    // return the cost.
    return double( t.cost_ );
  }

  //create the trajectories we wish to score
  Trajectory TrajectoryPlanner::createTrajectories(double x, double y, double theta, 
      double vx, double vy, double vtheta,
      double acc_x, double acc_y, double acc_theta){
    //compute feasible velocity limits in robot space
    double max_vel_x, max_vel_theta;
    double min_vel_x, min_vel_theta;
    // ackerman variables
    double v_t;
    double steer_angle;
    double min_steer=0,max_steer=0;
      
    double dvx;
    double dvtheta;
 
    // should we use the ackerman configuration?
    /* transform the general status to the ackerman status */
    v_t=vx;
    if(vy>this->ack_steer_angle_max_)
      vy=this->ack_steer_angle_max_;
    else if(vy<this->ack_steer_angle_min_)
      vy=this->ack_steer_angle_min_;
    steer_angle=vy;
    if(vtheta>this->ack_steer_speed_max_)
      vtheta=this->ack_steer_speed_max_;
    else if(vtheta<this->ack_steer_speed_min_)
      vtheta=this->ack_steer_speed_min_;
    this->steering_speed_=vtheta;
      /* compute the margins */
    //should we use the dynamic window approach?
    double T4=0.0;

    // compute the simulation time
    double dist=sqrt((global_plan_[global_plan_.size()-1].pose.position.x-x)*(global_plan_[global_plan_.size()-1].pose.position.x-x)+
                     (global_plan_[global_plan_.size()-1].pose.position.y-y)*(global_plan_[global_plan_.size()-1].pose.position.y-y));
    sim_time_=dist/ack_vel_max_;
    if(sim_time_>10.0)
      sim_time_=10.0;
    else if(sim_time_<3.0)
      sim_time_=3.0;
    ROS_WARN("Simulation time %f\n",sim_time_);
    // compute the trajectory times
    if(this->steering_speed_>=0)
    {
      max_vel_theta=(this->steering_speed_+this->ack_steer_acc_max_*sim_time_)/2.0;
      if(max_vel_theta>=this->ack_steer_speed_max_)
      {
        max_vel_theta=this->ack_steer_speed_max_;  
        T4=sim_time_-(2.0*max_vel_theta-this->steering_speed_)/this->ack_steer_acc_max_;
        max_steer=max_vel_theta*max_vel_theta/this->ack_steer_acc_max_-this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)+max_vel_theta*T4+steer_angle;
        if(max_steer>this->ack_steer_angle_max_) 
        {
          max_steer=this->ack_steer_angle_max_;
          T4=(this->ack_steer_angle_max_-steer_angle-max_vel_theta*max_vel_theta/this->ack_steer_acc_max_+this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_))/max_vel_theta;
          if(T4<0)
          {
            T4=0;
            max_vel_theta=sqrt(this->steering_speed_*this->steering_speed_/2.0+this->ack_steer_acc_max_*(this->ack_steer_angle_max_-steer_angle));
          }
        }
      }
      else
      { 
        max_steer=max_vel_theta*max_vel_theta/this->ack_steer_acc_max_-this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)+steer_angle;
        T4=0;
        if(max_steer>this->ack_steer_angle_max_)
        {
          max_steer=this->ack_steer_angle_max_;
          max_vel_theta=sqrt(this->steering_speed_*this->steering_speed_/2.0+this->ack_steer_acc_max_*(this->ack_steer_angle_max_-steer_angle));
        }
      }
      min_vel_theta=(this->steering_speed_-this->ack_steer_acc_max_*sim_time_)/2.0;
      if(min_vel_theta<=this->ack_steer_speed_min_)
      {
        min_vel_theta=this->ack_steer_speed_min_;
        T4=sim_time_+(2.0*min_vel_theta-this->steering_speed_)/this->ack_steer_acc_max_;
        min_steer=this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)-min_vel_theta*min_vel_theta/this->ack_steer_acc_max_+min_vel_theta*T4+steer_angle;
        if(min_steer<this->ack_steer_angle_min_)
        {
          min_steer=this->ack_steer_angle_min_;
          T4=(this->ack_steer_angle_min_-steer_angle+min_vel_theta*min_vel_theta/this->ack_steer_acc_max_-this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_))/min_vel_theta;
          if(T4<0)
          {
            T4=0;
            min_vel_theta=-sqrt(this->steering_speed_*this->steering_speed_/2.0-this->ack_steer_acc_max_*(this->ack_steer_angle_min_-steer_angle));
          }
        }
      }
      else
      {
        min_steer=this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)-min_vel_theta*min_vel_theta/this->ack_steer_acc_max_+steer_angle;
        T4=0;
        if(min_steer<this->ack_steer_angle_min_)
        {
          min_steer=this->ack_steer_angle_min_;
          min_vel_theta=-sqrt(this->steering_speed_*this->steering_speed_/2.0-this->ack_steer_acc_max_*(this->ack_steer_angle_min_-steer_angle)); 
        }
      }
    }
    else
    {
      max_vel_theta=(this->steering_speed_+this->ack_steer_acc_max_*sim_time_)/2.0;
      if(max_vel_theta>=this->ack_steer_speed_max_)
      {
        max_vel_theta=this->ack_steer_speed_max_;
        T4=sim_time_-(2.0*max_vel_theta-this->steering_speed_)/this->ack_steer_acc_max_;
        max_steer=-this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)+max_vel_theta*max_vel_theta/this->ack_steer_acc_max_+max_vel_theta*T4+steer_angle;
        if(max_steer>this->ack_steer_angle_max_)
        {
          max_steer=this->ack_steer_angle_max_;
          T4=(this->ack_steer_angle_max_-steer_angle-max_vel_theta*max_vel_theta/this->ack_steer_acc_max_+this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_))/max_vel_theta;
          if(T4<0)
          {
            T4=0;
            max_vel_theta=sqrt(this->steering_speed_*this->steering_speed_/2.0+this->ack_steer_acc_max_*(this->ack_steer_angle_max_-steer_angle));
          }
        }
      }
      else
      {
        max_steer=-this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)+max_vel_theta*max_vel_theta/this->ack_steer_acc_max_+steer_angle;
        T4=0;
        if(max_steer>this->ack_steer_angle_max_)
        {
          max_steer=this->ack_steer_angle_max_;
          max_vel_theta=sqrt(this->steering_speed_*this->steering_speed_/2.0+this->ack_steer_acc_max_*(this->ack_steer_angle_max_-steer_angle));
        }
      }
      min_vel_theta=(this->steering_speed_-this->ack_steer_acc_max_*sim_time_)/2.0;
      if(min_vel_theta<=this->ack_steer_speed_min_)
      {
        min_vel_theta=this->ack_steer_speed_min_;
        T4=sim_time_+(2.0*min_vel_theta-this->steering_speed_)/this->ack_steer_acc_max_;
        min_steer=-min_vel_theta*min_vel_theta/this->ack_steer_acc_max_+this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)+min_vel_theta*T4+steer_angle;
        if(min_steer<this->ack_steer_angle_min_)
        {
          min_steer=this->ack_steer_angle_min_;
          T4=(this->ack_steer_angle_min_-steer_angle+min_vel_theta*min_vel_theta/this->ack_steer_acc_max_-this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_))/min_vel_theta;
          if(T4<0)
          {
            T4=0;
            min_vel_theta=-sqrt(this->steering_speed_*this->steering_speed_/2.0-this->ack_steer_acc_max_*(this->ack_steer_angle_min_-steer_angle));
          }
        }
      }
      else
      {
        min_steer=-min_vel_theta*min_vel_theta/this->ack_steer_acc_max_+this->steering_speed_*this->steering_speed_/(2.0*this->ack_steer_acc_max_)+steer_angle;
        T4=0;
        if(min_steer<this->ack_steer_angle_min_)
        {
          min_steer=this->ack_steer_angle_min_;
          min_vel_theta=-sqrt(this->steering_speed_*this->steering_speed_/2.0-this->ack_steer_acc_max_*(this->ack_steer_angle_min_-steer_angle)); 
        }
      }
    }

    dist=sqrt((map_.goal_x_-x)*(map_.goal_x_-x)+(map_.goal_y_-y)*(map_.goal_y_-y));
    double d=0.0,T5=0.0,a,b,c;

    max_vel_x=(v_t+this->ack_acc_max_*sim_time_)/2.0;
    if(max_vel_x>this->ack_vel_max_)
      max_vel_x=this->ack_vel_max_;
    if(v_t>0)
    {
      T5=sim_time_-(2.0*max_vel_x-v_t)/this->ack_acc_max_;
      d=max_vel_x*max_vel_x/this->ack_acc_max_-v_t*v_t/(2.0*this->ack_acc_max_)+max_vel_x*T5;
      if(d>dist)
      {
        d=dist;
        a=1;
        b=-v_t-sim_time_*this->ack_acc_max_;
        c=v_t*v_t/2.0+dist*this->ack_acc_max_;
        max_vel_x=(-b-sqrt(b*b-4*a*c))/(2.0*a);
      }
    }
    else
    {
      T5=sim_time_-(2.0*max_vel_x-v_t)/this->ack_acc_max_;
      d=-v_t*v_t/(2.0*this->ack_acc_max_)+max_vel_x*max_vel_x/this->ack_acc_max_+max_vel_x*T5;
      if(d>dist)
      {
        d=dist;
        a=1;
        b=-v_t-sim_time_*this->ack_acc_max_;
        c=v_t*v_t/2.0+dist*this->ack_acc_max_;
        max_vel_x=(-b-sqrt(b*b-4*a*c))/(2.0*a);
      }
    }
    min_vel_x=-max_vel_x;
    ROS_INFO("TrajectoryPlanner::createTrajectories: Goal_dist: %f, Dist: %f, Max_vel: %f", dist, d, max_vel_x);
      //we want to sample the velocity space regularly
    dvx = (max_vel_x - min_vel_x) / (vx_samples_ - 1);
    dvtheta = (max_steer - min_steer) / (vtheta_samples_ - 1);

    double vx_samp = min_vel_x;
    double vtheta_samp = min_steer;
    double vy_samp = 0.0;

    //keep track of the best trajectory seen so far
    Trajectory* best_traj = &traj_one;
    best_traj->cost_ = -1.0;

    Trajectory* comp_traj = &traj_two;
    comp_traj->cost_ = -1.0;

    Trajectory* swap = NULL;

    //any cell with a cost greater than the size of the map is impossible
    double impossible_cost = map_.map_.size();

    ROS_INFO("TrajectoryPlanner::createTrajectories: CurrentSteerAngle: %f, currentSteerSpeed: %f, CurrentSpeed: %f", vy, vtheta, vx);
    //loop through all x velocities
    for(int i = 0; i < vx_samples_; ++i)
    {
      vtheta_samp = 0;
      /* compute */
      //first sample the straight trajectory
      generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
          acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);

      //if the new trajectory is better... let's take it
      if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
        swap = best_traj;
        best_traj = comp_traj;
        comp_traj = swap;
        ROS_DEBUG("TrajectoryPlanner::createTrajectories: vt: %f, SteerAngle: 0, Cost: %f", i*dvx+min_vel_x, comp_traj->cost_);
      }

      vtheta_samp = min_steer;
      //next sample all theta trajectories
      for(int j = 0; j < vtheta_samples_ ; ++j){
        generateTrajectory(x, y, theta, vx, vy, vtheta, vx_samp, vy_samp, vtheta_samp, 
            acc_x, acc_y, acc_theta, impossible_cost, *comp_traj);
        //if the new trajectory is better... let's take it
        if(comp_traj->cost_ >= 0 && (comp_traj->cost_ <= best_traj->cost_ || best_traj->cost_ < 0)){
          swap = best_traj;
          best_traj = comp_traj;
          comp_traj = swap;
          ROS_DEBUG("TrajectoryPlanner::createTrajectories: vt: %f, SteerAngle: %f, Cost: %f", i*dvx + min_vel_x, j*dvtheta + min_steer, comp_traj->cost_);
        }
        vtheta_samp += dvtheta;
      }
      vx_samp += dvx;
    }

    //next we want to generate trajectories for rotating in place
    vtheta_samp = min_vel_theta;
    vx_samp = 0.0;
    vy_samp = 0.0;

    //do we have a legal trajectory
    if(best_traj->cost_ >= 0)
    {
      /* if velocity command is small and the distance to goal is bigger than the threshold -> replan */
      dist=sqrt((map_.goal_x_-x)*(map_.goal_x_-x)+(map_.goal_y_-y)*(map_.goal_y_-y));
      if(dist>xy_goal_tol_ && fabs(best_traj->xv_)<0.02)
        best_traj->cost_=-1;
      return *best_traj;
    }

    //if the trajectory failed because the footprint hits something, we're still going to back up
    if(best_traj->cost_ == -1.0)
    {
      best_traj->resetPoints();
      best_traj->xv_ = 0.0;
      best_traj->yv_ = 0.0;
      best_traj->thetav_ = 0.0;
      best_traj->cost_ = -3.0;// no solution, keep the previous command
    }

    return *best_traj;
  }

  //given the current state of the robot, find a good trajectory
  Trajectory TrajectoryPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose, tf::Stamped<tf::Pose> global_vel, 
      tf::Stamped<tf::Pose>& drive_velocities,geometry_msgs::Twist &ackermann_state)
  {
    static Trajectory last_best_traj;

    double yaw = tf::getYaw(global_pose.getRotation());
    //double vel_yaw = tf::getYaw(global_vel.getRotation());

    double x = global_pose.getOrigin().getX();
    double y = global_pose.getOrigin().getY();
    double theta = yaw;

    double vx = ackermann_state.linear.z;
    double vy = ackermann_state.angular.x;
    double vtheta = ackermann_state.angular.y;

    //reset the map for new operations
    map_.resetPathDist();

    //temporarily remove obstacles that are within the footprint of the robot
    vector<iri_ackermann_local_planner::Position2DInt> footprint_list = getFootprintCells(x, y, theta, true);

    //mark cells within the initial footprint of the robot
    for(unsigned int i = 0; i < footprint_list.size(); ++i){
      map_(footprint_list[i].x, footprint_list[i].y).within_robot = true;
    }

    //make sure that we update our path based on the global plan and compute costs
    map_.setPathCells(costmap_, global_plan_);
    ROS_DEBUG("Path/Goal distance computed");


    //rollout trajectories and find the minimum cost one
    Trajectory best = createTrajectories(x, y, theta, 
        vx, vy, vtheta, 
        ack_acc_max_, ack_vel_max_, ack_vel_min_);
    ROS_DEBUG("Trajectories created");

    if(best.cost_ < 0){
      if(best.cost_==-3)
      {
        best.xv_=last_best_traj.xv_;
        best.yv_=last_best_traj.yv_;
        best.thetav_=last_best_traj.thetav_;
        best.cost_=last_best_traj.cost_;
        tf::Vector3 start(best.xv_, best.yv_, 0);
        drive_velocities.setOrigin(start);
        tf::Matrix3x3 matrix;
        matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
        drive_velocities.setBasis(matrix);
      }
      else
        drive_velocities.setIdentity();
    }
    else{
      tf::Vector3 start(best.xv_, best.yv_, 0);
      drive_velocities.setOrigin(start);
      tf::Matrix3x3 matrix;
      matrix.setRotation(tf::createQuaternionFromYaw(best.thetav_));
      drive_velocities.setBasis(matrix);
      last_best_traj.xv_=best.xv_;
      last_best_traj.yv_=best.yv_;
      last_best_traj.thetav_=best.thetav_;
      last_best_traj.cost_=best.cost_;
    }

    return best;
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double TrajectoryPlanner::footprintCost(double x_i, double y_i, double theta_i){
    //build the oriented footprint
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    vector<geometry_msgs::Point> oriented_footprint;
    for(unsigned int i = 0; i < footprint_spec_.size(); ++i){
      geometry_msgs::Point new_pt;
      new_pt.x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_pt.y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      oriented_footprint.push_back(new_pt);
    }

    geometry_msgs::Point robot_position;
    robot_position.x = x_i;
    robot_position.y = y_i;

    //check if the footprint is legal
    double footprint_cost = world_model_.footprintCost(robot_position, oriented_footprint, costmap_.getInscribedRadius(), costmap_.getCircumscribedRadius());

    return footprint_cost;
  }

  void TrajectoryPlanner::getLineCells(int x0, int x1, int y0, int y1, vector<iri_ackermann_local_planner::Position2DInt>& pts){
    //Bresenham Ray-Tracing
    int deltax = abs(x1 - x0);        // The difference between the x's
    int deltay = abs(y1 - y0);        // The difference between the y's
    int x = x0;                       // Start x off at the first pixel
    int y = y0;                       // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    iri_ackermann_local_planner::Position2DInt pt;

    if (x1 >= x0)                 // The x-values are increasing
    {
      xinc1 = 1;
      xinc2 = 1;
    }
    else                          // The x-values are decreasing
    {
      xinc1 = -1;
      xinc2 = -1;
    }

    if (y1 >= y0)                 // The y-values are increasing
    {
      yinc1 = 1;
      yinc2 = 1;
    }
    else                          // The y-values are decreasing
    {
      yinc1 = -1;
      yinc2 = -1;
    }

    if (deltax >= deltay)         // There is at least one x-value for every y-value
    {
      xinc1 = 0;                  // Don't change the x when numerator >= denominator
      yinc2 = 0;                  // Don't change the y for every iteration
      den = deltax;
      num = deltax / 2;
      numadd = deltay;
      numpixels = deltax;         // There are more x-values than y-values
    }
    else                          // There is at least one y-value for every x-value
    {
      xinc2 = 0;                  // Don't change the x for every iteration
      yinc1 = 0;                  // Don't change the y when numerator >= denominator
      den = deltay;
      num = deltay / 2;
      numadd = deltax;
      numpixels = deltay;         // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
      pt.x = x;      //Draw the current pixel
      pt.y = y;
      pts.push_back(pt);

      num += numadd;              // Increase the numerator by the top of the fraction
      if (num >= den)             // Check if numerator >= denominator
      {
        num -= den;               // Calculate the new numerator value
        x += xinc1;               // Change the x as appropriate
        y += yinc1;               // Change the y as appropriate
      }
      x += xinc2;                 // Change the x as appropriate
      y += yinc2;                 // Change the y as appropriate
    }
  }

  //get the cellsof a footprint at a given position
  vector<iri_ackermann_local_planner::Position2DInt> TrajectoryPlanner::getFootprintCells(double x_i, double y_i, double theta_i, bool fill){
    vector<iri_ackermann_local_planner::Position2DInt> footprint_cells;

    //if we have no footprint... do nothing
    if(footprint_spec_.size() <= 1){
      unsigned int mx, my;
      if(costmap_.worldToMap(x_i, y_i, mx, my)){
        Position2DInt center;
        center.x = mx;
        center.y = my;
        footprint_cells.push_back(center);
      }
      return footprint_cells;
    }

    //pre-compute cos and sin values
    double cos_th = cos(theta_i);
    double sin_th = sin(theta_i);
    double new_x, new_y;
    unsigned int x0, y0, x1, y1;
    unsigned int last_index = footprint_spec_.size() - 1;

    for(unsigned int i = 0; i < last_index; ++i){
      //find the cell coordinates of the first segment point
      new_x = x_i + (footprint_spec_[i].x * cos_th - footprint_spec_[i].y * sin_th);
      new_y = y_i + (footprint_spec_[i].x * sin_th + footprint_spec_[i].y * cos_th);
      if(!costmap_.worldToMap(new_x, new_y, x0, y0))
        return footprint_cells;

      //find the cell coordinates of the second segment point
      new_x = x_i + (footprint_spec_[i + 1].x * cos_th - footprint_spec_[i + 1].y * sin_th);
      new_y = y_i + (footprint_spec_[i + 1].x * sin_th + footprint_spec_[i + 1].y * cos_th);
      if(!costmap_.worldToMap(new_x, new_y, x1, y1))
        return footprint_cells;

      getLineCells(x0, x1, y0, y1, footprint_cells);
    }

    //we need to close the loop, so we also have to raytrace from the last pt to first pt
    new_x = x_i + (footprint_spec_[last_index].x * cos_th - footprint_spec_[last_index].y * sin_th);
    new_y = y_i + (footprint_spec_[last_index].x * sin_th + footprint_spec_[last_index].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x0, y0))
      return footprint_cells;

    new_x = x_i + (footprint_spec_[0].x * cos_th - footprint_spec_[0].y * sin_th);
    new_y = y_i + (footprint_spec_[0].x * sin_th + footprint_spec_[0].y * cos_th);
    if(!costmap_.worldToMap(new_x, new_y, x1, y1))
      return footprint_cells;

    getLineCells(x0, x1, y0, y1, footprint_cells);

    if(fill)
      getFillCells(footprint_cells);

    return footprint_cells;
  }

  void TrajectoryPlanner::getFillCells(vector<iri_ackermann_local_planner::Position2DInt>& footprint){
    //quick bubble sort to sort pts by x
    iri_ackermann_local_planner::Position2DInt swap, pt;
    unsigned int i = 0;
    while(i < footprint.size() - 1){
      if(footprint[i].x > footprint[i + 1].x){
        swap = footprint[i];
        footprint[i] = footprint[i + 1];
        footprint[i + 1] = swap;
        if(i > 0)
          --i;
      }
      else
        ++i;
    }

    i = 0;
    iri_ackermann_local_planner::Position2DInt min_pt;
    iri_ackermann_local_planner::Position2DInt max_pt;
    unsigned int min_x = footprint[0].x;
    unsigned int max_x = footprint[footprint.size() -1].x;
    //walk through each column and mark cells inside the footprint
    for(unsigned int x = min_x; x <= max_x; ++x){
      if(i >= footprint.size() - 1)
        break;

      if(footprint[i].y < footprint[i + 1].y){
        min_pt = footprint[i];
        max_pt = footprint[i + 1];
      }
      else{
        min_pt = footprint[i + 1];
        max_pt = footprint[i];
      }

      i += 2;
      while(i < footprint.size() && footprint[i].x == x){
        if(footprint[i].y < min_pt.y)
          min_pt = footprint[i];
        else if(footprint[i].y > max_pt.y)
          max_pt = footprint[i];
        ++i;
      }

      //loop though cells in the column
      for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
        pt.x = x;
        pt.y = y;
        footprint.push_back(pt);
      }
    }
  }

  void TrajectoryPlanner::getLocalGoal(double& x, double& y){
    x = map_.goal_x_;
    y = map_.goal_y_;
  }

};
