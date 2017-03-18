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

#include <moveit/move_group_interface/move_group_interface.h>

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

  ros::Publisher target_pose_pub_;
  ros::Subscriber pick_and_place_sub_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;

  // Pick and place parameters
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double attach_time;
  double detach_time;
  double z_up;

public:
  PickAndPlaceServer(const std::string name) :
    nh_("~"), as_(name, false), action_name_(name), arm_("arm"), gripper_("gripper")
  {
    // Read specific pick and place parameters
    nh_.param("grasp_attach_time", attach_time, 0.8);
    nh_.param("grasp_detach_time", detach_time, 0.6);

    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));

    as_.start();

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[pick and place] Received goal!");
    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;

    arm_.setPoseReferenceFrame(arm_link);

    // Allow some leeway in position (meters) and orientation (radians)
    arm_.setGoalPositionTolerance(0.001);
    arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);

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
    ros::Duration(attach_time).sleep(); // ensure that gripper properly grasp the cube before lifting the arm

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
    ros::Duration(detach_time).sleep(); // ensure that gripper properly release the cube before lifting the arm

    /* go up */
    target.position.z = z_up;
    if (moveArmTo(target) == false)
      return;

    as_.setSucceeded(result_);
  }

private:
  /**
   * Move arm to a named configuration, normally described in the robot semantic description SRDF file.
   * @param target Named target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const std::string& target)
  {
    ROS_DEBUG("[pick and place] Move arm to '%s' position", target.c_str());
    if (arm_.setNamedTarget(target) == false)
    {
      ROS_ERROR("[pick and place] Set named target '%s' failed", target.c_str());
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = arm_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[pick and place] Move to target failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }

  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::Pose& target)
  {
    int attempts = 0;
    ROS_DEBUG("[pick and place] Move arm to [%.2f, %.2f, %.2f, %.2f]",
             target.position.x, target.position.y, target.position.z, tf::getYaw(target.orientation));
    while (attempts < 5)
    {
      geometry_msgs::PoseStamped modiff_target;
      modiff_target.header.frame_id = arm_link;
      modiff_target.pose = target;

      double x = modiff_target.pose.position.x;
      double y = modiff_target.pose.position.y;
      double z = modiff_target.pose.position.z;
      double d = sqrt(x*x + y*y);
      if (d > 0.3)
      {
        // Maximum reachable distance by the turtlebot arm is 30 cm
        ROS_ERROR("Target pose out of reach [%f > %f]", d, 0.3);
        as_.setAborted(result_);
        return false;
      }
      // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
      // we point the gripper (0.205 = arm's max reach - vertical pitch distance + ε). Yaw is the direction to
      // the target. We also try some random variations of both to increase the chances of successful planning.
      // Roll is simply ignored, as our arm lacks the proper dof.
      double rp = M_PI_2 - std::asin((d - 0.1)/0.205) + attempts*fRand(-0.05, +0.05);
      double ry = std::atan2(y, x) + attempts*fRand(-0.05, +0.05);
      double rr = 0.0;

      tf::Quaternion q = tf::createQuaternionFromRPY(rr, rp, ry);
      tf::quaternionTFToMsg(q, modiff_target.pose.orientation);

      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner
      ROS_DEBUG("z increase:  %f  +  %f", modiff_target.pose.position.z, std::abs(std::cos(rp))/50.0);
      modiff_target.pose.position.z += std::abs(std::cos(rp))/50.0;

      ROS_DEBUG("Set pose target [%.2f, %.2f, %.2f] [d: %.2f, r: %.2f, p: %.2f, y: %.2f]", x, y, z, d, rr, rp, ry);
      target_pose_pub_.publish(modiff_target);

      if (arm_.setPoseTarget(modiff_target) == false)
      {
        ROS_ERROR("Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                  tf::getYaw(modiff_target.pose.orientation));
        as_.setAborted(result_);
        return false;
      }

      moveit::planning_interface::MoveItErrorCode result = arm_.move();
      if (bool(result) == true)
      {
        return true;
      }
      else
      {
        ROS_ERROR("[pick and place] Move to target failed (error %d) at attempt %d",
                  result.val, attempts + 1);
      }
      attempts++;
    }

    ROS_ERROR("[pick and place] Move to target failed after %d attempts", attempts);
    as_.setAborted(result_);
    return false;
  }

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @return True of success, false otherwise
   */
  bool setGripper(float opening)
  {
    ROS_DEBUG("[pick and place] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[pick and place] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = gripper_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[pick and place] Set gripper opening failed (error %d)", result.val);
      as_.setAborted(result_);
      return false;
    }
  }

  float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  }
};

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_action_server");

  turtlebot_arm_block_manipulation::PickAndPlaceServer server("pick_and_place");

  // Setup an multi-threaded spinner as the move groups operations need continuous spinning
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
