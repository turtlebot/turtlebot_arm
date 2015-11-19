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

#include <yocs_math_toolkit/common.hpp>

#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseArray.h>

#include <actionlib/server/simple_action_server.h>
#include <turtlebot_arm_object_manipulation/PickAndPlaceAction.h>
#include <turtlebot_arm_object_manipulation/MoveToTargetAction.h>

#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


namespace turtlebot_arm_object_manipulation
{

class PickAndPlaceServer
{
private:
  actionlib::SimpleActionServer<turtlebot_arm_object_manipulation::PickAndPlaceAction> as_;
  std::string action_name_;

  turtlebot_arm_object_manipulation::PickAndPlaceFeedback     feedback_;
  turtlebot_arm_object_manipulation::PickAndPlaceResult       result_;
  turtlebot_arm_object_manipulation::PickAndPlaceGoalConstPtr goal_;

  ros::Publisher target_pose_pub_;
  ros::Subscriber planning_scene_sub_;

  std_srvs::Empty empty_srv_;
  ros::ServiceClient clear_octomap_srv_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroup arm_;
  moveit::planning_interface::MoveGroup gripper_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  std::vector<moveit_msgs::AttachedCollisionObject> attached_collision_objs_;

  // Pick and place parameters
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double attach_time;
  double detach_time;
  double z_backlash;

  const int PICK_ATTEMPTS = 10;
  const int PLACE_ATTEMPTS = PICK_ATTEMPTS;

public:
  PickAndPlaceServer(const std::string name) :
    as_(name, false), action_name_(name), arm_("arm"), gripper_("gripper")
  {
    ros::NodeHandle nh("~");

    // Read specific pick and place parameters
    nh.param("grasp_attach_time", attach_time, 0.8);
    nh.param("grasp_detach_time", detach_time, 0.6);
    nh.param("vertical_backlash", z_backlash, 0.01);

    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));

    as_.start();

    // We will clear the octomap and retry whenever a pick/place fails
    clear_octomap_srv_ = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

    // We subscribe to planning scene to keep track of attached/detached objects
    planning_scene_sub_ = nh.subscribe("/move_group/monitored_planning_scene", 1, &PickAndPlaceServer::sceneCB, this);

    // We publish the pick and place poses for debugging purposes
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);
  }

  ~PickAndPlaceServer()
  {
    as_.shutdown();
  }

  void goalCB()
  {
    ROS_INFO("[pick and place] Received goal!");

    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;

    arm_.setPoseReferenceFrame(arm_link);
    arm_.setSupportSurfaceName("table");

    // Allow some leeway in position (meters) and orientation (radians)
    arm_.setGoalPositionTolerance(0.001);
    arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);

    pickAndPlace(goal_->obj_name, goal_->pick_pose, goal_->place_pose);
  }

  void preemptCB()
  {
    ROS_WARN("[pick and place] %s: Preempted", action_name_.c_str());
    gripper_.stop();
    arm_.stop();

    // set the action state to preempted
    as_.setPreempted();
  }

  void sceneCB(const moveit_msgs::PlanningScene& scene)
  {
    // Keep attached collision objects so we can subtract its pose from the place location poses
    attached_collision_objs_ = scene.robot_state.attached_collision_objects;
  }

  bool pickAndPlace(const std::string& obj_name, const geometry_msgs::Pose& pick_pose,
                                                 const geometry_msgs::Pose& place_pose)
  {
    if (pick(obj_name, pick_pose))
    {
      if (place(obj_name, place_pose))
      {
        as_.setSucceeded(result_);
        return true;
      }
      else
      {
        // Ensure we don't retain any object attached to the gripper
        arm_.detachObject(obj_name);
        setGripper(gripper_open);
      }
    }

    as_.setAborted(result_);
    return false;
  }

  bool pick(const std::string& obj_name, const geometry_msgs::Pose& pose)
  {
    ROS_INFO("[pick and place] Picking...");

    // Try up to PICK_ATTEMPTS grasps with slightly different poses
    for (int attempt = 0; attempt < PICK_ATTEMPTS; ++attempt)
    {
      geometry_msgs::PoseStamped p;
      p.pose = pose;

      if (!validateTargetPose(p, attempt))
      {
        return false;
      }

      moveit_msgs::Grasp g;
      g.grasp_pose = p;

      g.pre_grasp_approach.direction.vector.x = 0.5;
      g.pre_grasp_approach.direction.header.frame_id = arm_.getEndEffectorLink();
      g.pre_grasp_approach.min_distance = 0.005;
      g.pre_grasp_approach.desired_distance = 0.1;

      g.post_grasp_retreat.direction.header.frame_id = arm_.getEndEffectorLink();
      g.post_grasp_retreat.direction.vector.x = -0.5;
      g.post_grasp_retreat.min_distance = 0.005;
      g.post_grasp_retreat.desired_distance = 0.1;

      g.pre_grasp_posture.joint_names.resize(1, "gripper_joint");
      g.pre_grasp_posture.points.resize(1);
      g.pre_grasp_posture.points[0].positions.resize(1);
      g.pre_grasp_posture.points[0].positions[0] = gripper_open;

      g.grasp_posture.joint_names.resize(1, "gripper_joint");
      g.grasp_posture.points.resize(1);
      g.grasp_posture.points[0].positions.resize(1);
      g.grasp_posture.points[0].positions[0] = gripper_closed;

      g.allowed_touch_objects.push_back("table");

      g.id = attempt;

      std::vector<moveit_msgs::Grasp> grasps(1, g);

      if (arm_.pick(obj_name, grasps))
      {
        ROS_INFO("[pick and place] Pick successfully completed");
        return true;
      }
      
      if (attempt == 1)
        clear_octomap_srv_.call(empty_srv_);
    }

    ROS_ERROR("[pick and place] Pick failed after %d attempts", PICK_ATTEMPTS);
    return false;
  }

  bool place(const std::string& obj_name, const geometry_msgs::Pose& pose)
  {
    ROS_INFO("[pick and place] Placing...");

    // Try up to PLACE_ATTEMPTS place locations with slightly different poses
    for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
    {
      geometry_msgs::PoseStamped p;
      p.pose = pose;

      if (!validateTargetPose(p, attempt))
      {
        return false;
      }

      if (attached_collision_objs_.size() > 0)
      {
        // MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
        // the orientation it had when picked. However, with our 4-dofs arm this is infeasible (and also we don't care
        // about the objects orientation), so we cancel this transformation. It is applied here:
        // https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
        // More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
        geometry_msgs::Pose aco_pose = attached_collision_objs_[0].object.primitive_poses[0];

        tf::Transform place_tf, aco_tf;
        tf::poseMsgToTF(p.pose, place_tf);
        tf::poseMsgToTF(aco_pose, aco_tf);
        tf::poseTFToMsg(place_tf * aco_tf, p.pose);

        ROS_DEBUG("Compensate place pose with the attached object pose [%s]. Results: [%s]",
                  mtk::pose2str3D(aco_pose).c_str(), mtk::pose2str3D(p.pose).c_str());
      }

      moveit_msgs::PlaceLocation l;
      l.place_pose = p;

      l.pre_place_approach.direction.vector.x = 0.5;
      l.pre_place_approach.direction.header.frame_id = arm_.getEndEffectorLink();
      l.pre_place_approach.min_distance = 0.005;
      l.pre_place_approach.desired_distance = 0.1;

      l.post_place_retreat.direction.vector.x = -0.5;
      l.post_place_retreat.direction.header.frame_id = arm_.getEndEffectorLink();
      l.post_place_retreat.min_distance = 0.005;
      l.post_place_retreat.desired_distance = 0.1;

      l.post_place_posture.joint_names.resize(1, "gripper_joint");
      l.post_place_posture.points.resize(1);
      l.post_place_posture.points[0].positions.resize(1);
      l.post_place_posture.points[0].positions[0] = gripper_open;

      l.allowed_touch_objects.push_back("table");

      l.id = attempt;

      std::vector<moveit_msgs::PlaceLocation> locs(1, l);

      if (arm_.place(obj_name, locs))
      {
        ROS_INFO("[pick and place] Place successfully completed");
        return true;
      }

      if (attempt == 1)
        clear_octomap_srv_.call(empty_srv_);
    }

    ROS_ERROR("[pick and place] Place failed after %d attempts", PLACE_ATTEMPTS);
    return false;
  }

private:
  /**
   * Convert a simple 3D point into a valid pick/place pose. The orientation Euler angles
   * are calculated as a function of the x and y coordinates, plus some random variations
   * Increasing with the number of attempts to improve our chances of successful planning.
   * @param target Pose target to validate
   * @param attempt The actual attempts number
   * @return True of success, false otherwise
   */
  bool validateTargetPose(geometry_msgs::PoseStamped& target, int attempt = 0)
  {
    target.header.frame_id = arm_link;

    double x = target.pose.position.x;
    double y = target.pose.position.y;
    double z = target.pose.position.z;
    double d = sqrt(x*x + y*y);
    if (d > 0.3)
    {
      // Maximum reachable distance by the turtlebot arm is 30 cm, but above twenty something the arm makes
      // strange and ugly contortions, and overcomes the reduced elbow lower limit we have to operate always
      // with the same gripper orientation
      // XXX solved constraining also both shoulder limits (180 deg. operation); we get back the 30 cm limit
      ROS_ERROR("[pick and place] Target pose out of reach [%f > %f]", d, 0.3);
      return false;
    }

    // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
    // we point the gripper (0.22 = arm's max reach - vertical pitch distance + ε). Yaw is the direction to
    // the target. We also try some random variations of both to increase the chances of successful planning.
    // Roll is plainly ignored, as our arm lacks that dof.
    double rp = M_PI_2 - std::asin((d - 0.1)/0.22) + ((attempt%2)*2 - 1)*(std::ceil(attempt/2.0)*0.05);
    double ry = std::atan2(y, x) + ((attempt%2)*2 - 1)*(std::ceil(attempt/2.0)*0.05);
    double rr = 0.0;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry);

    // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner and
    // a bit extra to compensate the effect of the arm's backlash in the height of the gripper over the table
    double z_delta1 = std::abs(std::cos(rp))/50.0;
    double z_delta2 = z_backlash;
    ROS_DEBUG("[pick and place] Z increase:  %f  +  %f  +  %f", target.pose.position.z, z_delta1, z_delta2);
    target.pose.position.z += z_delta1;
    target.pose.position.z += z_delta2;

    ROS_DEBUG("[pick and place] Set pose target [%s] [d: %.2f]", mtk::pose2str3D(target.pose).c_str(), d);
    target_pose_pub_.publish(target);

    return true;
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
      return false;
    }
  }

  float fRand(float min, float max)
  {
    return ((float(rand()) / float(RAND_MAX)) * (max - min)) + min;
  }
};


/**
 * Action server providing more basic arm motions:
 *  - move to a named target
 *  - move to a joint state configuration
 *  - move to a particular pose target
 */
class MoveToTargetServer
{
private:
  actionlib::SimpleActionServer<turtlebot_arm_object_manipulation::MoveToTargetAction> as_;
  std::string action_name_;

  turtlebot_arm_object_manipulation::MoveToTargetFeedback     feedback_;
  turtlebot_arm_object_manipulation::MoveToTargetResult       result_;
  turtlebot_arm_object_manipulation::MoveToTargetGoalConstPtr goal_;

  ros::Publisher target_pose_pub_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroup arm_;
  moveit::planning_interface::MoveGroup gripper_;

public:
  MoveToTargetServer(const std::string name) :
    as_(name, false), action_name_(name), arm_("arm"), gripper_("gripper")
  {
    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&MoveToTargetServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveToTargetServer::preemptCB, this));

    as_.start();

    ros::NodeHandle nh("~");
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);
  }

  ~MoveToTargetServer()
  {
    as_.shutdown();
  }

  void goalCB()
  {
    ROS_INFO("[move to target] Received goal!");
    goal_ = as_.acceptNewGoal();
    bool result = false;

    switch (goal_->target_type)
    {
      case turtlebot_arm_object_manipulation::MoveToTargetGoal::NAMED_TARGET:
        result = moveArmTo(goal_->named_target);
        break;
      case turtlebot_arm_object_manipulation::MoveToTargetGoal::JOINT_STATE:
      case turtlebot_arm_object_manipulation::MoveToTargetGoal::POSE_TARGET:
      default:
        ROS_ERROR("[move to target] Move to target of type %d not implemented", goal_->target_type);
        break;
    }

    if (result)
    {
      as_.setSucceeded(result_);
    }
    else
    {
      as_.setAborted(result_);
    }
  }

  void preemptCB()
  {
    ROS_INFO("[move to target] %s: Preempted", action_name_.c_str());
    gripper_.stop();
    arm_.stop();

    // set the action state to preempted
    as_.setPreempted();
  }

private:
  /**
   * Move arm to a named configuration, normally described in the robot semantic description SRDF file.
   * @param target Named target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const std::string& target)
  {
    if (target == "resting")  // XXX temporal cheat... blocks less fov to the camera
      setGripper(0.002);

    ROS_DEBUG("[move to target] Move arm to '%s' position", target.c_str());
    if (arm_.setNamedTarget(target) == false)
    {
      ROS_ERROR("[move to target] Set named target '%s' failed", target.c_str());
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = arm_.move();
    if (bool(result) == true)
    {
      ROS_INFO("[move to target] Move to target \"%s\" completed", target.c_str());
      return true;
    }
    else
    {
      ROS_ERROR("[move to target] Move to target \"%s\" failed (error %d)", target.c_str(), result.val);
      return false;
    }
  }

  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool moveArmTo(const geometry_msgs::PoseStamped& target)
  {
    int attempts = 0;
    ROS_DEBUG("[move to target] Move arm to [%.2f, %.2f, %.2f, %.2f]",
             target.pose.position.x, target.pose.position.y, target.pose.position.z,
             tf::getYaw(target.pose.orientation));
    while (attempts < 5)
    {
      geometry_msgs::PoseStamped modiff_target = target;

      double x = modiff_target.pose.position.x;
      double y = modiff_target.pose.position.y;
      double z = modiff_target.pose.position.z;
      double d = sqrt(x*x + y*y);
      if (d > 0.3)
      {
        // Maximum reachable distance by the turtlebot arm is 30 cm
        ROS_ERROR("[move to target] Target pose out of reach [%f > %f]", d, 0.3);
        return false;
      }
      // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
      // we point the gripper. Yaw is the direction to the target. We also try some random variations of both to
      // increase the chances of successful planning.
      double rp = M_PI_2 - std::asin((d - 0.1)/0.205); // 0.205 = arm's max reach - vertical pitch distance + ε
      double ry = std::atan2(y, x);

      tf::Quaternion q = tf::createQuaternionFromRPY(0.0,
                                                     attempts*fRand(-0.05, +0.05) + rp,
                                                     attempts*fRand(-0.05, +0.05) + ry);
      tf::quaternionTFToMsg(q, modiff_target.pose.orientation);

      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner
      ROS_DEBUG("[move to target] Z increase:  %f  +  %f", modiff_target.pose.position.z, std::abs(std::cos(rp))/50.0);
      modiff_target.pose.position.z += std::abs(std::cos(rp))/50.0;

      ROS_DEBUG("[move to target] Set pose target [%.2f, %.2f, %.2f] [d: %.2f, p: %.2f, y: %.2f]", x, y, z, d, rp, ry);
      target_pose_pub_.publish(modiff_target);

      if (arm_.setPoseTarget(modiff_target) == false)
      {
        ROS_ERROR("[move to target] Set pose target [%.2f, %.2f, %.2f, %.2f] failed",
                  modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                  tf::getYaw(modiff_target.pose.orientation));
        return false;
      }

      moveit::planning_interface::MoveItErrorCode result = arm_.move();
      if (bool(result) == true)
      {
        ROS_INFO("[move to target] Move to target [%.2f, %.2f, %.2f, %.2f] completed",
                 modiff_target.pose.position.x, modiff_target.pose.position.y, modiff_target.pose.position.z,
                 tf::getYaw(modiff_target.pose.orientation));
        return true;
      }
      else
      {
        ROS_ERROR("[move to target] Move to target failed (error %d) at attempt %d",
                  result.val, attempts + 1);
      }
      attempts++;
    }

    ROS_ERROR("[move to target] Move to target failed after %d attempts", attempts);
    return false;
  }

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @return True of success, false otherwise
   */
  bool setGripper(float opening)
  {
    ROS_DEBUG("[move to target] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[move to target] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result = gripper_.move();
    if (bool(result) == true)
    {
      return true;
    }
    else
    {
      ROS_ERROR("[move to target] Set gripper opening failed (error %d)", result.val);
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

  // Create pick_and_place and move_to_target action servers
  turtlebot_arm_object_manipulation::PickAndPlaceServer pnp_server("pick_and_place");
  turtlebot_arm_object_manipulation::MoveToTargetServer mtt_server("move_to_target");

  // Setup an asynchronous spinner as the move groups operations need continuous spinning
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();
  spinner.stop();

  return 0;
}
