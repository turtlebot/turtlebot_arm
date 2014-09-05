/* 
 * Copyright (c) 2011, Vanadium Labs LLC
 * All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Vanadium Labs LLC nor the names of its 
 *       contributors may be used to endorse or promote products derived 
 *       from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Michael Ferguson, Helen Oleynikova
 */

#include <ros/ros.h>
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <turtlebot_arm_block_manipulation/PickAndPlaceAction.h>

#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/PoseArray.h>

namespace turtlebot_arm_block_manipulation
{

class PickAndPlaceServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_arm_block_manipulation::PickAndPlaceAction> as_;
  std::string action_name_;
  
  turtlebot_arm_block_manipulation::PickAndPlaceFeedback     feedback_;
  turtlebot_arm_block_manipulation::PickAndPlaceResult       result_;
  turtlebot_arm_block_manipulation::PickAndPlaceGoalConstPtr goal_;
  
  moveit::planning_interface::MoveGroup arm_;
  moveit::planning_interface::MoveGroup gripper_;

  ros::Subscriber pick_and_place_sub_;
  
  // Parameters from goal
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;
public:
  PickAndPlaceServer(const std::string name) : 
    nh_("~"), as_(name, false), action_name_(name), arm_("arm"), gripper_("gripper")
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));
    
    as_.start();
  }

  void goalCB()
  {
    ROS_INFO("[pick and place] Received goal!");
    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;
    
    if (goal_->topic.length() < 1)
      pickAndPlace(goal_->pickup_pose, goal_->place_pose);
    else
      pick_and_place_sub_ = nh_.subscribe(goal_->topic, 1, &PickAndPlaceServer::sendGoalFromTopic, this);
  }

  void sendGoalFromTopic(const geometry_msgs::PoseArrayConstPtr& msg)
  {
    ROS_INFO("[pick and place] Got goal from topic! %s", goal_->topic.c_str());
    pickAndPlace(msg->poses[0], msg->poses[1]);
    pick_and_place_sub_.shutdown();
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
  
  void pickAndPlace(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    ROS_INFO("[pick and place] Picking. Also placing.");
  
    geometry_msgs::Pose target;

    /* open gripper */
    if (setGripper(gripper_open) == false)
      return;
    
    /* arm straight up */
    if (moveArmTo(target, "right_up") == false)
      return;

    /* hover over */
    target = start_pose;
    target.position.z = z_up;
    if (moveArmTo(target) == false)
      return;

    /* go down */
    target.position.z = start_pose.position.z;
    if (moveArmTo(target) == false)
      return;

    /* close gripper */
    if (setGripper(gripper_closed) == false)
      return;

    /* go up */
    target.position.z = z_up;
    if (moveArmTo(target) == false)
      return;

    /* hover over */
    target = end_pose;
    target.position.z = z_up;
    if (moveArmTo(target) == false)
      return;

    /* go down */
    target.position.z = end_pose.position.z;
    if (moveArmTo(target) == false)
      return;

    /* open gripper */
    if (setGripper(gripper_open) == false)
      return;

    /* go up */
    target.position.z = z_up;
    if (moveArmTo(target) == false)
      return;
  }

  bool moveArmTo(const geometry_msgs::Pose& target, const std::string& named_target = "")
  {
    if (named_target.length() == 0)
    {
      ROS_DEBUG("[pick and place] Move arm to [%.2f, %.2f, %.2f]",
               target.position.x, target.position.y, target.position.z);
      if (arm_.setPoseTarget(target) == false)
      {
        ROS_ERROR("Set pose target [%.2f, %.2f, %.2f] failed",
                  target.position.x, target.position.y, target.position.z);
        return false;
      }
    }
    else
    {
      ROS_DEBUG("[pick and place] Move arm to %s", named_target.c_str());
      if (arm_.setNamedTarget(named_target) == false)
      {
        ROS_ERROR("[pick and place] Set named target '%s' failed", named_target.c_str());
        return false;
      }
    }

    moveit_msgs::MoveItErrorCodes result = arm_.move();
    if (result == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      as_.setSucceeded(result_);
      return true;
    }
    else
    {
      ROS_ERROR("[pick and place] Move to target failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }

  bool setGripper(float opening)
  {
    ROS_DEBUG("[pick and place] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[pick and place] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit_msgs::MoveItErrorCodes result = gripper_.move();
    if (result == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      as_.setSucceeded(result_);
      return true;
    }
    else
    {
      ROS_ERROR("[pick and place] Set gripper opening failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }
};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_action_server");

  turtlebot_arm_block_manipulation::PickAndPlaceServer server("pick_and_place");
  ros::spin();

  return 0;
}

