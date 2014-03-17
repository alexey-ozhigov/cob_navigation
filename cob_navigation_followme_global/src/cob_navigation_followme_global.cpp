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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
* Authors: Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include <cob_navigation_followme_global/cob_navigation_followme_global.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <sstream>
#include <math.h>
#include <stdlib.h>

using namespace std;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(cob_navigation_followme_global::COBGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace cob_navigation_followme_global {
  string Goal2Str(const geometry_msgs::PoseStamped& goal){
      stringstream sstr;
      sstr << '('
          << goal.pose.position.x << ", "
          << goal.pose.position.y << ", "
          << goal.pose.position.z << ", "
          << goal.pose.orientation.x << ", "
          << goal.pose.orientation.y << ", "
          << goal.pose.orientation.z << ", "
          << goal.pose.orientation.w << ") ";
      return sstr.str();
  }
  string Plan2Str(const std::vector<geometry_msgs::PoseStamped>& plan){
    stringstream sstr;
    for (int i = 0; i < plan.size(); i++)
        sstr << '('
             << plan[i].pose.position.x << ", "
             << plan[i].pose.position.y << ", "
             << plan[i].pose.position.z << ", "
             << plan[i].pose.orientation.x << ", "
             << plan[i].pose.orientation.y << ", "
             << plan[i].pose.orientation.z << ", "
             << plan[i].pose.orientation.w << ") ";
    sstr << endl;
    return sstr.str();
  }

  void makeLinearPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan){

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;
    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = goal_x;
    double target_y = goal_y;
    double target_yaw = goal_yaw;


    int num_points = 10;
    float step_x = diff_x / num_points;
    float step_y = diff_y / num_points;
    float step_yaw = diff_yaw / num_points;

    for (int i = 0; i < num_points; i++)
    {
        target_x = start_x + i * step_x;
        target_y = start_y + i * step_y;
        target_yaw = angles::normalize_angle(start_yaw + i * step_yaw);
        geometry_msgs::PoseStamped point = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

        point.pose.position.x = target_x;
        point.pose.position.y = target_y;

        point.pose.orientation.x = goal_quat.x();
        point.pose.orientation.y = goal_quat.y();
        point.pose.orientation.z = goal_quat.z();
        point.pose.orientation.w = goal_quat.w();

        plan.push_back(point);
        /*
           double footprint_cost = footprintCost(target_x, target_y, target_yaw);
           if(footprint_cost >= 0)
           {
           done = true;
           }
         */
    }
  }

  void COBGlobalPlanner::makePlanFromTracker(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    plan.clear();
    for (int i = 0; i < poses.size(); i++) {
        geometry_msgs::PoseStamped point = goal;
        point.pose.position = poses[i].pose.position;
        if (i == poses.size() - 1)
            point.pose.orientation = poses[i - 1].pose.orientation;
        else {
            const geometry_msgs::Point& prev_pose = poses[i - 1].pose.position;
            const geometry_msgs::Point& cur_pose = point.pose.position;
            geometry_msgs::Quaternion& cur_orient = point.pose.orientation;
            float dx = prev_pose.x - cur_pose.x;
            float dy = prev_pose.y - cur_pose.y;
            float yaw = atan2f(dy, dx);
            tf::Quaternion quat = tf::createQuaternionFromYaw(yaw); 
            cur_orient.x = quat.x();
            cur_orient.y = quat.y();
            cur_orient.z = quat.z();
            cur_orient.w = quat.w();
            plan.push_back(point);
        }
    }
  }
 
  void COBGlobalPlanner::poseCB(const geometry_msgs::PoseStamped& pose){
      ROS_INFO("GlobalPlanner received pose %s", Goal2Str(pose).c_str());
      poses.push_back(pose);
  }
  COBGlobalPlanner::COBGlobalPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  COBGlobalPlanner::COBGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  
  void COBGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      world_model_ = new base_local_planner::CostmapModel(*costmap_); 
      pose_topic = private_nh.subscribe(FOLLOWME_POSE_TOPIC, 1, &COBGlobalPlanner::poseCB, this);
      path_log_topic = private_nh.advertise<nav_msgs::Path>(FOLLOWME_PATH_LOG_TOPIC, 5);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double COBGlobalPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }


  bool COBGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    static int call_cnt;
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, must call initialize() first");
      return false;
    }

    ROS_INFO("Plan task: start (%.2f, %.2f) goal (%.2f, %.2f); poses received: %lu",
             start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y, poses.size());

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();
    string global_frame = costmap_ros_->getGlobalFrameID();

    if(goal.header.frame_id != global_frame){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          global_frame.c_str(), goal.header.frame_id.c_str());
      return false;
    }

    if (poses.size() == 0)
        makeLinearPlan(start, goal, plan);
    else
        makePlanFromTracker(start, goal, plan);
    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    nav_msgs::Path path_log_msg;
    path_log_msg.header.stamp = ros::Time::now();
    path_log_msg.poses = plan;
    path_log_topic.publish(path_log_msg);
    string plan_str = Plan2Str(plan);
    ROS_INFO("Plan (len %lu): %s", plan.size(), plan_str.c_str());

    return true;
  }

};
