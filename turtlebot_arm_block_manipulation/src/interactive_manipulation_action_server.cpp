/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Michael Ferguson, Helen Oleynikova
 */

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>

#include <turtlebot_arm_block_manipulation/InteractiveBlockManipulationAction.h>
#include <geometry_msgs/PoseArray.h>

using namespace visualization_msgs;

namespace turtlebot_arm_block_manipulation
{

class InteractiveManipulationServer
{
private:
  ros::NodeHandle nh_;

  interactive_markers::InteractiveMarkerServer server_;
  
  actionlib::SimpleActionServer<turtlebot_arm_block_manipulation::InteractiveBlockManipulationAction> as_;
  std::string action_name_;
  
  turtlebot_arm_block_manipulation::InteractiveBlockManipulationFeedback     feedback_;
  turtlebot_arm_block_manipulation::InteractiveBlockManipulationResult       result_;
  turtlebot_arm_block_manipulation::InteractiveBlockManipulationGoalConstPtr goal_;
  
  ros::Subscriber block_sub_;
  ros::Publisher  pick_and_place_pub_;

  geometry_msgs::Pose old_pose_;
  
  geometry_msgs::PoseArrayConstPtr msg_;
  bool initialized_;
  
  // Parameters from goal
  std::string arm_link;
  double      block_size;
  
  // Parameters from server
  double bump_size;

public:

  InteractiveManipulationServer(const std::string name) : 
     nh_("~"), server_("block_controls"), as_(name, false), action_name_(name), initialized_(false), block_size(0)
  {
    // Load parameters from the server.
    nh_.param<double>("bump_size", bump_size, 0.005);
    
    // Register the goal and feeback callbacks.
    as_.registerGoalCallback(boost::bind(&InteractiveManipulationServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&InteractiveManipulationServer::preemptCB, this));
    
    as_.start();
  
    block_sub_ = nh_.subscribe("/turtlebot_blocks", 1, &InteractiveManipulationServer::addBlocks, this);
    pick_and_place_pub_ = nh_.advertise< geometry_msgs::PoseArray >("/pick_and_place", 1, true);
  }
  
  void goalCB()
  {

    // accept the new goal
    goal_ = as_.acceptNewGoal();
    
    ROS_INFO("[interactive manipulation] Received goal! %f, %s", goal_->block_size, goal_->frame.c_str());
    
    block_size = goal_->block_size;
    arm_link = goal_->frame;
    
    if (initialized_)
      addBlocks(msg_);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
  
  void addBlocks(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    server_.clear();
    server_.applyChanges();
  
    ROS_INFO("Got block detection callback. Adding blocks.");
    geometry_msgs::Pose block;
    bool active = as_.isActive();
    
    for (unsigned int i=0; i < msg->poses.size(); i++)
    {
      block = msg->poses[i];
      addBlock(block, i, active, msg->header.frame_id);
      ROS_INFO("Added %d blocks", i);
    }
    
    server_.applyChanges();
    
    msg_ = msg;
    initialized_ = true;
  }
  

  // Move the real block!
  void feedbackCb( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    if (!as_.isActive())
    {
      ROS_INFO("Got feedback but not active!");
      return;
    }
    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM("Staging " << feedback->marker_name);     
        old_pose_ = feedback->pose;
        break;
   
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM("Now moving " << feedback->marker_name); 
        moveBlock(old_pose_, feedback->pose);
        break;
    }
    
    server_.applyChanges(); 
  }
  
  void moveBlock(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    // Return pickup and place poses as the result of the action
    geometry_msgs::Pose start_pose_bumped, end_pose_bumped;
    start_pose_bumped = start_pose;
    //  start_pose_bumped.position.y -= bump_size;  TODO/WARN: this was in the original version, but destroys piss off indigo/moveit operation!
    start_pose_bumped.position.z -= block_size/2.0 - bump_size;

    result_.pickup_pose = start_pose_bumped;
    
    end_pose_bumped = end_pose;
    end_pose_bumped.position.z -= block_size/2.0 - bump_size;
    result_.place_pose = end_pose_bumped;
    
    // Publish pickup and place poses for visualizing on RViz
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = arm_link;
    msg.header.stamp = ros::Time::now();
    msg.poses.push_back(start_pose_bumped);
    msg.poses.push_back(end_pose_bumped);
    
    pick_and_place_pub_.publish(msg);
    
    as_.setSucceeded(result_);
    
    server_.clear();
    server_.applyChanges();
  }

  // Make a box
  Marker makeBox( InteractiveMarker &msg, float r, float g, float b )
  {
    Marker m;

    m.type = Marker::CUBE;
    m.scale.x = msg.scale;
    m.scale.y = msg.scale;
    m.scale.z = msg.scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    return m;
  }
   
  // Add a new block
  void addBlock( const geometry_msgs::Pose pose, int n, bool active, std::string link)
  {
    InteractiveMarker marker;
    marker.header.frame_id = link;
    marker.pose = pose;
    marker.scale = block_size;
    
    std::stringstream conv;
    conv << n;
    conv.str();
     
    marker.name = "block" + conv.str();

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    
    if (active)
      marker.controls.push_back( control );
    
    control.markers.push_back( makeBox(marker, .5, .5, .5) );
    control.always_visible = true;
    marker.controls.push_back( control );
    

    server_.insert( marker );
    server_.setCallback( marker.name, boost::bind( &InteractiveManipulationServer::feedbackCb, this, _1 ));
  }

};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "interactive_manipulation_action_server");

  turtlebot_arm_block_manipulation::InteractiveManipulationServer manip("interactive_manipulation");

  ros::spin();
}

