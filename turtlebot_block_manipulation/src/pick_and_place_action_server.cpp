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
#include <turtlebot_block_manipulation/PickAndPlaceAction.h>

#include <actionlib/client/simple_action_client.h>
#include <simple_arm_server/MoveArmAction.h>

#include <geometry_msgs/PoseArray.h>

namespace turtlebot_block_manipulation
{

class PickAndPlaceServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_block_manipulation::PickAndPlaceAction> as_;
  std::string action_name_;
  
  turtlebot_block_manipulation::PickAndPlaceFeedback     feedback_;
  turtlebot_block_manipulation::PickAndPlaceResult       result_;
  turtlebot_block_manipulation::PickAndPlaceGoalConstPtr goal_;
  
  actionlib::SimpleActionClient<simple_arm_server::MoveArmAction> client_;
  
  ros::Subscriber pick_and_place_sub_;
  
  // Parameters from goal
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double z_up;
public:
  PickAndPlaceServer(const std::string name) : 
    nh_("~"), as_(name, false), action_name_(name), client_("/move_arm", true)
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
  
    simple_arm_server::MoveArmGoal goal;
    simple_arm_server::ArmAction action;
    simple_arm_server::ArmAction grip;
    
    /* open gripper */
    grip.type = simple_arm_server::ArmAction::MOVE_GRIPPER;
    grip.move_time.sec = 1.0;
    grip.command = gripper_open;
    goal.motions.push_back(grip);
    
    /* arm straight up */
    tf::Quaternion temp;
    temp.setRPY(0,1.57,0);
    action.goal.orientation.x = temp.getX();
    action.goal.orientation.y = temp.getY();
    action.goal.orientation.z = temp.getZ();
    action.goal.orientation.w = temp.getW();

    /* hover over */
    action.goal.position.x = start_pose.position.x;
    action.goal.position.y = start_pose.position.y;
    action.goal.position.z = z_up;
    action.move_time.sec = 0.25;
    goal.motions.push_back(action);

    /* go down */
    action.goal.position.z = start_pose.position.z;
    action.move_time.sec = 1.5;
    goal.motions.push_back(action);

    /* close gripper */
    grip.type = simple_arm_server::ArmAction::MOVE_GRIPPER;
    grip.command = gripper_closed;
    grip.move_time.sec = 1.0;
    goal.motions.push_back(grip);

    /* go up */
    action.goal.position.z = z_up;
    action.move_time.sec = 1.0;
    goal.motions.push_back(action);

    /* hover over */
    action.goal.position.x = end_pose.position.x;
    action.goal.position.y = end_pose.position.y;
    action.goal.position.z = z_up;
    action.move_time.sec = 1.0;
    goal.motions.push_back(action);

    /* go down */
    action.goal.position.z = end_pose.position.z;
    action.move_time.sec = 1.5;
    goal.motions.push_back(action);

    /* open gripper */
    grip.command = gripper_open;
    goal.motions.push_back(grip);

    /* go up */
    action.goal.position.z = z_up;
    action.move_time.sec = 1.0;
    goal.motions.push_back(action);
  
    goal.header.frame_id = arm_link;
    client_.sendGoal(goal);
    ROS_INFO("[pick and place] Sent goal. Waiting.");
    client_.waitForResult(ros::Duration(30.0));
    ROS_INFO("[pick and place] Received result.");
    if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      as_.setSucceeded(result_);
    else
    {
      ROS_INFO("Actual state: %s", client_.getState().toString().c_str());
      as_.setAborted(result_);
    }
  }
};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_action_server");

  turtlebot_block_manipulation::PickAndPlaceServer server("pick_and_place");
  ros::spin();

  return 0;
}

