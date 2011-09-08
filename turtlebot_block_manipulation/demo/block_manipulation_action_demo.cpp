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
#include <turtlebot_block_manipulation/BlockDetectionAction.h>
#include <turtlebot_block_manipulation/PickAndPlaceAction.h>
#include <turtlebot_block_manipulation/InteractiveBlockManipulationAction.h>
#include <simple_arm_actions/ResetArmAction.h>

#include <string>
#include <sstream>


const std::string arm_link = "/arm_base_link";
const double gripper_open = 0.042;
const double gripper_closed = 0.024;

const double z_up = 0.12;
double z_down = 0.01;

const double block_size = 0.03;

const std::string pick_and_place_topic = "/pick_and_place";

namespace turtlebot_block_manipulation
{

class BlockManipulationAction
{
private:
  ros::NodeHandle nh_;
  
  // Actions
  actionlib::SimpleActionClient<BlockDetectionAction> block_detection_action_;
  actionlib::SimpleActionClient<InteractiveBlockManipulationAction> interactive_manipulation_action_;
  actionlib::SimpleActionClient<PickAndPlaceAction> pick_and_place_action_;
  actionlib::SimpleActionClient<simple_arm_actions::ResetArmAction> reset_arm_action_;
  
  BlockDetectionGoal block_detection_goal_;
  InteractiveBlockManipulationGoal interactive_manipulation_goal_;
  PickAndPlaceGoal pick_and_place_goal_;
  

public:

  BlockManipulationAction() : 
    block_detection_action_("block_detection", true),
    interactive_manipulation_action_("interactive_manipulation", true),
    pick_and_place_action_("pick_and_place", true),
    reset_arm_action_("reset_arm", true)
  {
    // Initialize goals
    block_detection_goal_.frame = arm_link;
    block_detection_goal_.table_height = z_down;
    block_detection_goal_.block_size = block_size;
    
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.z_down = z_down;
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
    
    reset_arm_action_.sendGoal(simple_arm_actions::ResetArmGoal());
    
    detectBlocks();
  }
  
  void detectBlocks()
  {
    block_detection_action_.sendGoal(block_detection_goal_,  boost::bind( &BlockManipulationAction::addBlocks, this, _1, _2));
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
      
    reset_arm_action_.sendGoal(simple_arm_actions::ResetArmGoal());
    ros::shutdown();
  }
};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");
  if (argc == 2)
  {
    // Convert the first command-line argument to a double and store it as the 
    // table height.
    std::string argument(argv[1]);
    
    std::stringstream stream(argument);
    stream >> z_down;
    ROS_INFO("Setting the table height to %f", z_down);
  }


  turtlebot_block_manipulation::BlockManipulationAction manip;

  // everything is done in cloud callback, just spin
  ros::spin();
}

