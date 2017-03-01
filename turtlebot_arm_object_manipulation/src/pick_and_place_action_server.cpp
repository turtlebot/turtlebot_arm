/*
 * Copyright (c) 2015, Jorge Santos
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
 * Author: Jorge Santos
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>
#include <geometric_shapes/shape_operations.h>

// action servers
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_arm_object_manipulation/PickAndPlaceAction.h>
#include <turtlebot_arm_object_manipulation/MoveToTargetAction.h>

// MoveIt!
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>
#include <moveit/move_group_interface/move_group_interface.h>
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

  std_srvs::Empty empty_srv_;
  ros::Publisher target_pose_pub_;
  ros::ServiceClient clear_octomap_srv_;

  tf::TransformListener tf_listener_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;

  // We use the planning scene to gather information of tabletop/attached objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // Pick and place parameters
  std::string arm_link;
  double gripper_open;
  double attach_time;
  double detach_time;
  double z_backlash;

  const int PICK_ATTEMPTS = 5;
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
    nh.param("/gripper_controller/max_opening", gripper_open, 0.045);

    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));

    as_.start();

    // We will clear the octomap and retry whenever a pick/place fails
    clear_octomap_srv_ = nh.serviceClient<std_srvs::Empty>("/clear_octomap");

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

    arm_.setPoseReferenceFrame(arm_link);
    arm_.setSupportSurfaceName("table");

    // Allow some leeway in position (meters) and orientation (radians)
    arm_.setGoalPositionTolerance(0.001);
    arm_.setGoalOrientationTolerance(0.02);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);

    geometry_msgs::PoseStamped pick_pose, place_pose;
    pick_pose.header = goal_->header;
    pick_pose.pose = goal_->pick_pose;
    place_pose.header = goal_->header;
    place_pose.pose = goal_->place_pose;

    pickAndPlace(goal_->obj_name, pick_pose, place_pose);
  }

  void preemptCB()
  {
    ROS_WARN("[pick and place] %s: Preempted", action_name_.c_str());
    gripper_.stop();
    arm_.stop();

    // set the action state to preempted
    as_.setPreempted();
  }

  bool pickAndPlace(const std::string& obj_name, geometry_msgs::PoseStamped& pick_pose,
                                                 geometry_msgs::PoseStamped& place_pose)
  {
    if (pick(obj_name, pick_pose))
    {
      if (place(obj_name, place_pose, pick_pose.pose.position.z))
      {
        as_.setSucceeded(result_);
        return true;
      }
      else
      {
        // Ensure we don't retain any object attached to the gripper
        arm_.detachObject(obj_name);
        setGripper(gripper_open, false);
      }
    }

    as_.setAborted(result_);
    return false;
  }

  bool pick(const std::string& obj_name, geometry_msgs::PoseStamped& pose)
  {
    // Look for obj_name in the list of available objects
    std::map<std::string, moveit_msgs::CollisionObject> objects =
        planning_scene_interface_.getObjects(std::vector<std::string>(1, obj_name));
    if (objects.size() == 0)
    {
      // Maybe the object's interactive marker name is wrong?
      ROS_ERROR("[pick and place] Tabletop collision object '%s' not found", obj_name.c_str());
      return false;
    }

    if (objects.size() > 1)
    {
      // This should not happen, as object detection tries to provide unique names to all objects...
      ROS_WARN("[pick and place] More than one (%d) tabletop collision objects with name '%s' found!",
               objects.size(), obj_name.c_str());
    }

    // We need object's pose and size for picking
    Eigen::Vector3d tco_size;
    geometry_msgs::PoseStamped tco_pose;
    const moveit_msgs::CollisionObject& tco = objects[obj_name];

    tco_pose.header = tco.header;

    // We get object's pose from the mesh/primitive poses; try first with the meshes
    if (tco.mesh_poses.size() > 0)
    {
      tco_pose.pose = tco.mesh_poses[0];
      if (tco.meshes.size() > 0)
      {
        tco_size = shapes::computeShapeExtents(tco.meshes[0]);

        // We assume meshes laying in the floor, so we bump its pose by half z-dimension to
        // grasp the object at mid-height. TODO: we could try something more sophisticated...
        tco_pose.pose.position.z += tco_size[2]/2.0;
      }
      else
      {
        ROS_ERROR("[pick and place] Tabletop collision object '%s' has no meshes", obj_name.c_str());
        return false;
      }
    }
    else if (tco.primitive_poses.size() > 0)
    {
      tco_pose.pose = tco.primitive_poses[0];
      if (tco.primitives.size() > 0)
      {
        tco_size = shapes::computeShapeExtents(tco.primitives[0]);
      }
      else
      {
        ROS_ERROR("[pick and place] Tabletop collision object '%s' has no primitives", obj_name.c_str());
        return false;
      }
    }
    else
    {
      ROS_ERROR("[pick and place] Tabletop collision object '%s' has no mesh/primitive poses", obj_name.c_str());
      return false;
    }

    ROS_INFO("[pick and place] Picking object '%s' with size [%.3f, %.3f, %.3f] at location [%s]...",
             obj_name.c_str(), tco_size[0], tco_size[1], tco_size[2], mtk::point2str2D(tco_pose.pose.position).c_str());

    // Try up to PICK_ATTEMPTS grasps with slightly different poses
    for (int attempt = 0; attempt < PICK_ATTEMPTS; ++attempt)
    {
      geometry_msgs::PoseStamped p = tco_pose;
      if (!validateTargetPose(p, true, attempt))
      {
        return false;
      }

      ROS_DEBUG("[pick and place] Pick attempt %d at pose [%s]...", attempt, mtk::pose2str3D(p).c_str());

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

      g.pre_grasp_posture.joint_names.push_back("gripper_joint");
      g.pre_grasp_posture.points.resize(1);
      g.pre_grasp_posture.points[0].positions.push_back(gripper_open);

      // As we grasp the object "blindly", just in the center, we use the maximum possible value as the opened
      // gripper position and the smallest dimension minus a small "tightening" epsilon as the closed position
      g.grasp_posture.joint_names.push_back("gripper_joint");
      g.grasp_posture.points.resize(1);
      g.grasp_posture.points[0].positions.push_back(tco_size.minCoeff() - 0.002);

      g.allowed_touch_objects.push_back(obj_name);
      g.allowed_touch_objects.push_back("table");

      g.id = attempt;

      std::vector<moveit_msgs::Grasp> grasps(1, g);

      moveit::planning_interface::MoveItErrorCode result = arm_.pick(obj_name, grasps);
      if (result)
      {
        ROS_INFO("[pick and place] Pick successfully completed");
        pose = p;  // provide the used pose so we use the same z coordinate
        return true;
      }

      ROS_DEBUG("[pick and place] Pick attempt %d failed: %s", attempt, mec2str(result));
      
      if (attempt == 1)
        clear_octomap_srv_.call(empty_srv_);
    }

    ROS_ERROR("[pick and place] Pick failed after %d attempts", PICK_ATTEMPTS);
    return false;
  }

  bool place(const std::string& obj_name, const geometry_msgs::PoseStamped& pose, double pick_position_z)
  {
    // Look for obj_name in the list of attached objects
    std::map<std::string, moveit_msgs::AttachedCollisionObject> objects =
        planning_scene_interface_.getAttachedObjects(std::vector<std::string>(1, obj_name));

    if (objects.size() == 0)
    {
      // Maybe pick failed; we will not continue because place will surely fail without knowing the attaching pose
      ROS_ERROR("[pick and place] Attached collision object '%s' not found", obj_name.c_str());
      return false;
    }

    if (objects.size() > 1)
    {
      // This should not happen... we grasped two objects with the same name???
      ROS_WARN("[pick and place] More than one (%d) attached collision objects with name '%s' found!",
               objects.size(), obj_name.c_str());
    }

    // We just need object's pose so we can subtract its pose from the place location poses
    geometry_msgs::Pose aco_pose;  // No stamped; it's relative to attaching frame (gripper_link)
    const moveit_msgs::AttachedCollisionObject& aco = objects[obj_name];

    if (aco.object.primitive_poses.size() > 0)
    {
      aco_pose = aco.object.primitive_poses[0];
    }
    else if (aco.object.mesh_poses.size() > 0)
    {
      aco_pose = aco.object.mesh_poses[0];
    }
    else
    {
      ROS_ERROR("[pick and place] Attached collision object '%s' has no pose!", obj_name.c_str());
      return false;
    }

    ROS_INFO("[pick and place] Placing object '%s' at pose [%s]...", obj_name.c_str(), mtk::pose2str3D(pose).c_str());

    // Try up to PLACE_ATTEMPTS place locations with slightly different poses
    for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
    {
      geometry_msgs::PoseStamped p = pose;
      if (!validateTargetPose(p, false, attempt))
      {
        return false;
      }

      // MoveGroup::place will transform the provided place pose with the attached body pose, so the object retains
      // the orientation it had when picked. However, with our 4-dofs arm this is infeasible (nor we care about the
      // objects orientation!), so we cancel this transformation. It is applied here:
      // https://github.com/ros-planning/moveit_ros/blob/jade-devel/manipulation/pick_place/src/place.cpp#L64
      // More details on this issue: https://github.com/ros-planning/moveit_ros/issues/577
      tf::Transform place_tf, aco_tf;
      tf::poseMsgToTF(p.pose, place_tf);
      tf::poseMsgToTF(aco_pose, aco_tf);
      tf::poseTFToMsg(place_tf * aco_tf, p.pose);
      p.pose.position.z = pick_position_z;

      ROS_DEBUG("Compensate place pose with the attached object pose [%s]. Results: [%s]",
                mtk::pose2str3D(aco_pose).c_str(), mtk::pose2str3D(p.pose).c_str());

      ROS_DEBUG("[pick and place] Place attempt %d at pose [%s]...", attempt, mtk::pose2str3D(p).c_str());

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

      l.post_place_posture.joint_names.push_back("gripper_joint");
      l.post_place_posture.points.resize(1);
      l.post_place_posture.points[0].positions.push_back(gripper_open);

      l.allowed_touch_objects.push_back(obj_name);
      l.allowed_touch_objects.push_back("table");

      l.id = attempt;

      std::vector<moveit_msgs::PlaceLocation> locs(1, l);

      moveit::planning_interface::MoveItErrorCode result = arm_.place(obj_name, locs);
      if (result)
      {
        ROS_INFO("[pick and place] Place successfully completed");
        return true;
      }

      ROS_DEBUG("[pick and place] Place attempt %d failed: %s", attempt, mec2str(result));

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
   * increasing with the number of attempts to improve our chances of successful planning.
   * @param target Pose target to validate
   * @param compensate_backlash Increment z to cope with backlash and low pitch poses
   * @param attempt The actual attempts number
   * @return True of success, false otherwise
   */
  bool validateTargetPose(geometry_msgs::PoseStamped& target, bool compensate_backlash, int attempt = 0)
  {
    // We always work relative to the arm base, so roll/pitch/yaw angles calculation make sense
    if (target.header.frame_id != arm_link)
    {
      transformPose(target.header.frame_id, arm_link, target, target);
    }

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
    // we point the gripper (0.22 = arm's max reach - vertical pitch distance + ε). We also try some random
    // variations to increase the chances of successful planning. Yaw is the direction to the target, and so
    // must be fixed. Roll is plainly ignored, as our arm lacks that dof.
    double rp = M_PI_2 - std::asin((d - 0.1)/0.22) + ((attempt%2)*2 - 1)*(std::ceil(attempt/2.0)*0.05);
    double ry = std::atan2(y, x);
    double rr = 0.0;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry);

    if (compensate_backlash)
    {
      // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner and
      // a bit extra to compensate the effect of the arm's backlash in the height of the gripper over the table
      double z_delta1 = std::abs(std::cos(rp))/50.0;
      double z_delta2 = z_backlash;
      ROS_DEBUG("[pick and place] Z increase:  %f  +  %f  +  %f", target.pose.position.z, z_delta1, z_delta2);
      target.pose.position.z += z_delta1;
      target.pose.position.z += z_delta2;
    }

    ROS_DEBUG("[pick and place] Target pose [%s] [d: %.2f]", mtk::pose2str3D(target.pose).c_str(), d);
    target_pose_pub_.publish(target);

    return true;
  }

  /**
   * Set gripper opening.
   * @param opening Physical opening of the gripper, in meters
   * @param wait_for_complete Wait or not for the execution of the trajectory to complete
   * @return True of success, false otherwise
   */
  bool setGripper(float opening, bool wait_for_complete = true)
  {
    ROS_DEBUG("[pick and place] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[pick and place] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result =
        wait_for_complete ? gripper_.move() : gripper_.asyncMove();
    if (result == true)
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

  const char* mec2str(const moveit::planning_interface::MoveItErrorCode& mec)
  {
    switch (mec.val)
    {
      case moveit::planning_interface::MoveItErrorCode::SUCCESS:
        return "success";
      case moveit::planning_interface::MoveItErrorCode::FAILURE:
        return "failure";
      case moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED:
        return "planning failed";
      case moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN:
        return "invalid motion plan";
      case moveit::planning_interface::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return "motion plan invalidated by environment change";
      case moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED:
        return "control failed";
      case moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
        return "unable to acquire sensor data";
      case moveit::planning_interface::MoveItErrorCode::TIMED_OUT:
        return "timed out";
      case moveit::planning_interface::MoveItErrorCode::PREEMPTED:
        return "preempted";
      case moveit::planning_interface::MoveItErrorCode::START_STATE_IN_COLLISION:
        return "start state in collision";
      case moveit::planning_interface::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return "start state violates path constraints";
      case moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION:
        return "goal in collision";
      case moveit::planning_interface::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
        return "goal violates path constraints";
      case moveit::planning_interface::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
        return "goal constraints violated";
      case moveit::planning_interface::MoveItErrorCode::INVALID_GROUP_NAME:
        return "invalid group name";
      case moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
        return "invalid goal constraints";
      case moveit::planning_interface::MoveItErrorCode::INVALID_ROBOT_STATE:
        return "invalid robot state";
      case moveit::planning_interface::MoveItErrorCode::INVALID_LINK_NAME:
        return "invalid link name";
      case moveit::planning_interface::MoveItErrorCode::INVALID_OBJECT_NAME:
        return "invalid object name";
      case moveit::planning_interface::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
        return "frame transform failure";
      case moveit::planning_interface::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
        return "collision checking unavailable";
      case moveit::planning_interface::MoveItErrorCode::ROBOT_STATE_STALE:
        return "robot state stale";
      case moveit::planning_interface::MoveItErrorCode::SENSOR_INFO_STALE:
        return "sensor info stale";
      case moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION:
        return "no ik solution";
      default:
        return "unrecognized error code";
    }
  }


  bool transformPose(const std::string& in_frame, const std::string& out_frame,
                     const geometry_msgs::PoseStamped& in_pose, geometry_msgs::PoseStamped& out_pose)
  {
    try
    {
      tf_listener_.waitForTransform(in_frame, out_frame, ros::Time(0.0), ros::Duration(1.0));
      tf_listener_.transformPose(out_frame, in_pose, out_pose);

      return true;
    }
    catch (tf::InvalidArgument& e)
    {
      ROS_ERROR("[pick and place] Transformed pose has invalid orientation: %s", e.what());
      return false;
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR("[pick and place] Could not get sensor to arm transform: %s", e.what());
      return false;
    }
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
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;

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
   * @param wait_for_complete Wait or not for the execution of the trajectory to complete
   * @return True of success, false otherwise
   */
  bool setGripper(float opening, bool wait_for_complete = true)
  {
    ROS_DEBUG("[move to target] Set gripper opening to %f", opening);
    if (gripper_.setJointValueTarget("gripper_joint", opening) == false)
    {
      ROS_ERROR("[move to target] Set gripper opening to %f failed", opening);
      return false;
    }

    moveit::planning_interface::MoveItErrorCode result =
        wait_for_complete ? gripper_.move() : gripper_.asyncMove();
    if (result == true)
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
  ros::init(argc, argv, "object_pick_and_place_action_server");

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
