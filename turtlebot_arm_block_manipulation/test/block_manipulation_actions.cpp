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
 */


#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <turtlebot_arm_block_manipulation/BlockDetectionAction.h>
#include <turtlebot_arm_block_manipulation/PickAndPlaceAction.h>

#include <interactive_markers/interactive_marker_server.h>


using namespace visualization_msgs;

const std::string arm_link = "/arm_base_link";
const double gripper_open = 0.042;
const double gripper_closed = 0.024;

const double z_up = 0.08;
const double z_down = -0.05;

const double block_size = 0.0127;


class BlockManipulationAction
{
private:
  interactive_markers::InteractiveMarkerServer server_;
  
  ros::NodeHandle nh_;
  
  // Actions
  actionlib::SimpleActionClient<turtlebot_arm_block_manipulation::BlockDetectionAction> block_detection_action_;
  actionlib::SimpleActionClient<turtlebot_arm_block_manipulation::PickAndPlaceAction> pick_and_place_action_;
  
  turtlebot_arm_block_manipulation::BlockDetectionGoal block_detection_goal_;
  turtlebot_arm_block_manipulation::PickAndPlaceGoal pick_and_place_goal_;
  
  geometry_msgs::Pose old_pose_;

public:

  BlockManipulationAction() : 
    server_("block_controls"), 
    block_detection_action_("block_detection", true),
    pick_and_place_action_("pick_and_place", true)
  {
    server_.applyChanges();
    
    // Initialize goals
    block_detection_goal_.frame = arm_link;
    block_detection_goal_.table_height = z_down;
    block_detection_goal_.block_size = block_size;
    
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.gripper_open = gripper_open;
    pick_and_place_goal_.gripper_closed = gripper_closed;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    block_detection_action_.waitForServer();
    pick_and_place_action_.waitForServer();
    
    ROS_INFO("Found servers.");
    
    detectBlocks();
  }
  
  void detectBlocks()
  {
    server_.clear();
    server_.applyChanges();
    
    block_detection_action_.sendGoal(block_detection_goal_,  boost::bind( &BlockManipulationAction::addBlocks, this, _1, _2));
  }
  
  void addBlocks(const actionlib::SimpleClientGoalState& state, const turtlebot_arm_block_manipulation::BlockDetectionResultConstPtr& result)
  {
    ROS_INFO("Got block detection callback. Adding blocks.");
    geometry_msgs::Pose block;
    
    for (unsigned int i=0; i < result->blocks.poses.size(); i++)
    {
      block = result->blocks.poses[i];
      addBlock(block.position.x, block.position.y, block.position.z, i);
      ROS_INFO("Added %d blocks", i);
    }
    
    server_.applyChanges(); 
  }
  

  // Move the real block!
  void feedbackCb( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM("Staging " << feedback->marker_name);     
          old_pose_ = feedback->pose;
        break;
   
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM("Now moving " << feedback->marker_name); 
        moveBlock(old_pose_, feedback->pose);
        detectBlocks();
        break;
    }
    
    server_.applyChanges(); 
  }
  
  bool moveBlock(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    pick_and_place_goal_.pickup_pose = start_pose;
    pick_and_place_goal_.place_pose = end_pose;
    pick_and_place_action_.sendGoalAndWait(pick_and_place_goal_, ros::Duration(30.0), ros::Duration(30.0));
    //pick_and_place_action_.sendGoal(pick_and_place_goal_);
    //pick_and_place_action_.waitForResult(ros::Duration(30.0));
    
    if ( block_detection_action_.getState()  == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return true;
    }
    else
    {
      return false;
    }
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
  void addBlock( float x, float y, float z, int n)
  {
    InteractiveMarker marker;
    marker.header.frame_id = arm_link;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale = 0.03;
    
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
    marker.controls.push_back( control );
    
    control.markers.push_back( makeBox(marker, .5, .5, .5) );
    control.always_visible = true;
    marker.controls.push_back( control );
    

    server_.insert( marker );
    server_.setCallback( marker.name, boost::bind( &BlockManipulationAction::feedbackCb, this, _1 ));
  }

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");

  BlockManipulationAction manip;

  // everything is done in cloud callback, just spin
  ros::spin();
}

