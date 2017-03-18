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
#include <turtlebot_arm_block_manipulation/InteractiveBlockManipulationAction.h>

#include <string>
#include <sstream>


const std::string pick_and_place_topic = "/pick_and_place";

namespace turtlebot_arm_block_manipulation
{

class BlockManipulationAction
{
private:
  ros::NodeHandle nh_;
  
  // Actions and services
  actionlib::SimpleActionClient<BlockDetectionAction> block_detection_action_;
  actionlib::SimpleActionClient<InteractiveBlockManipulationAction> interactive_manipulation_action_;
  actionlib::SimpleActionClient<PickAndPlaceAction> pick_and_place_action_;
  
  BlockDetectionGoal block_detection_goal_;
  InteractiveBlockManipulationGoal interactive_manipulation_goal_;
  PickAndPlaceGoal pick_and_place_goal_;

  // Parameters
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;
  double z_down;
  double block_size;
  bool once;
  
public:

  BlockManipulationAction() : nh_("~"),
    block_detection_action_("block_detection", true),
    interactive_manipulation_action_("interactive_manipulation", true),
    pick_and_place_action_("pick_and_place", true)
  {
    // Load parameters
    nh_.param<std::string>("arm_link", arm_link, "/arm_link");
    nh_.param<double>("gripper_open", gripper_open, 0.042);
    nh_.param<double>("gripper_closed", gripper_closed, 0.024);
    nh_.param<double>("z_up", z_up, 0.12);
    nh_.param<double>("table_height", z_down, 0.01);
    nh_.param<double>("block_size", block_size, 0.03);
    
    nh_.param<bool>("once", once, false);

    // Initialize goals
    block_detection_goal_.frame = arm_link;
    block_detection_goal_.table_height = z_down;
    block_detection_goal_.block_size = block_size;
    
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.gripper_open = gripper_open;
    pick_and_place_goal_.gripper_closed = gripper_closed;
    pick_and_place_goal_.topic = pick_and_place_topic;
    
    interactive_manipulation_goal_.block_size = block_size;
    interactive_manipulation_goal_.frame = arm_link;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    block_detection_action_.waitForServer();
    ROS_INFO("Found block detection server.");
    
    interactive_manipulation_action_.waitForServer();
    ROS_INFO("Found interactive manipulation.");
    
    pick_and_place_action_.waitForServer();
    ROS_INFO("Found pick and place server.");
    
    ROS_INFO("Found servers.");
  }
  
  void detectBlocks()
  {
    block_detection_action_.sendGoal(block_detection_goal_, boost::bind( &BlockManipulationAction::addBlocks, this, _1, _2));
  }
  
  void addBlocks(const actionlib::SimpleClientGoalState& state, const BlockDetectionResultConstPtr& result)
  {
    ROS_INFO("Got block detection callback. Adding blocks.");
    geometry_msgs::Pose block;
    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Succeeded!");
    else
    {
      ROS_INFO("Did not succeed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    interactive_manipulation_action_.sendGoal(interactive_manipulation_goal_, boost::bind( &BlockManipulationAction::pickAndPlace, this, _1, _2));
  }
  
  void pickAndPlace(const actionlib::SimpleClientGoalState& state, const InteractiveBlockManipulationResultConstPtr& result)
  {
    ROS_INFO("Got interactive marker callback. Picking and placing.");
    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Succeeded!");
    else
    {
      ROS_INFO("Did not succeed! %s",  state.toString().c_str());
      ros::shutdown();
    }
    pick_and_place_action_.sendGoal(pick_and_place_goal_, boost::bind( &BlockManipulationAction::finish, this, _1, _2));
  }
  
  void finish(const actionlib::SimpleClientGoalState& state, const PickAndPlaceResultConstPtr& result)
  {
    ROS_INFO("Got pick and place callback. Finished!");
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Succeeded!");
    else
      ROS_INFO("Did not succeed! %s",  state.toString().c_str());

    if (once)
      ros::shutdown();
    else
      detectBlocks();
  }
};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");

  turtlebot_arm_block_manipulation::BlockManipulationAction manip;

  // Setup an asynchronous spinner as the move groups operations need continuous spinning
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok())
  {
    manip.detectBlocks();

    // Allow user restarting, for the case that the block detection fails
    std::cout << "Press Enter for restarting block detection" << std::endl;
    std::cin.ignore();
  }

  spinner.stop();
}
