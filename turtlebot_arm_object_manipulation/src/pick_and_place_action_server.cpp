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
#include <turtlebot_arm_object_manipulation/PickAndPlaceAction.h>
#include <turtlebot_arm_object_manipulation/MoveToTargetAction.h>

#include <moveit_msgs/Grasp.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/PoseArray.h>

namespace turtlebot_arm_object_manipulation
{

class PickAndPlaceServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_arm_object_manipulation::PickAndPlaceAction> as_;
  std::string action_name_;

  turtlebot_arm_object_manipulation::PickAndPlaceFeedback     feedback_;
  turtlebot_arm_object_manipulation::PickAndPlaceResult       result_;
  turtlebot_arm_object_manipulation::PickAndPlaceGoalConstPtr goal_;

  ros::Publisher target_pose_pub_;
  ros::Subscriber pick_and_place_sub_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroup arm_;
  moveit::planning_interface::MoveGroup gripper_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

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
  PickAndPlaceServer(const std::string name,
                     const moveit::planning_interface::MoveGroup& arm,
                     const moveit::planning_interface::MoveGroup& gripper) :
    nh_("~"), as_(name, false), action_name_(name), arm_(arm), gripper_(gripper)
  {
    // Read specific pick and place parameters
    nh_.param("grasp_attach_time", attach_time, 0.8);
    nh_.param("grasp_detach_time", detach_time, 0.6);
    nh_.param("vertical_backlash", z_backlash, 0.01);

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


  std::string pose2str3D(const geometry_msgs::Pose& pose)
  {
    tf::Transform tf;
    tf::poseMsgToTF(pose, tf);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);

    char ___buffer___[100];
    sprintf(___buffer___, "%.2f, %.2f, %.2f,  %.2f, %.2f, %.2f",
            pose.position.x, pose.position.y, pose.position.z,
            roll, pitch, yaw);
    return std::string(___buffer___);
  }

//  0.17, -0.01, 0.04,  0.67, 0.84, 0.74
//  0.17, -0.01, 0.04,  1.30, -1.16, 1.75
//
//
//  0.18, -0.12, 0.05,  0.13,  1.00, -2.61
//  0.18, -0.12, 0.05, -0.98, -0.29,  1.76
//  0.18, -0.12, 0.05,  0.59, -0.88, -0.33
//  0.18, -0.12, 0.05,  0.93, -0.47, -0.89
//  0.18, -0.12, 0.05, -1.01, -0.12,  1.85
//  0.18, -0.12, 0.05, -0.52,  0.92,  2.89



  bool pickAndPlace(const std::string& obj_name, const geometry_msgs::Pose& pick_pose,
                                                 const geometry_msgs::Pose& place_pose)
  {
//    planning_scene_interface_. remove(std::vector<std::string>(1, co.id));
//    scene.remove_attached_object(GRIPPER_FRAME, "block_1")
    ROS_ERROR_STREAM(obj_name   << " object     " << arm_.getEndEffectorLink() << "   "<<arm_.getEndEffector() << "   " << planning_scene_interface_.getKnownObjectNames(false).size());

    std::map<std::string, geometry_msgs::Pose> ao_poses = planning_scene_interface_.getObjectPoses(std::vector<std::string>(1, obj_name));
    for (std::pair<std::string, geometry_msgs::Pose> ao_pose: ao_poses)
    {
      ROS_ERROR_STREAM("BEFORE PICK   " << ao_pose.first << " object at " << pose2str3D(ao_pose.second) << "        "<< arm_.getEndEffectorLink() << "   "<<arm_.getEndEffector());
    }


    if ((pick(obj_name, pick_pose)))////////////////////// && (place(obj_name, place_pose)))
    {
      std::map<std::string, geometry_msgs::Pose> ao_poses = planning_scene_interface_.getObjectPoses(std::vector<std::string>(1, obj_name));
      for (std::pair<std::string, geometry_msgs::Pose> ao_pose: ao_poses)
      {
        ROS_ERROR_STREAM("AFTER PICK    " << ao_pose.first << " object at " << pose2str3D(ao_pose.second) << "        "<< arm_.getEndEffectorLink() << "   "<<arm_.getEndEffector());
      }
      arm_.attachObject(obj_name, arm_.getEndEffectorLink());
      ao_poses = planning_scene_interface_.getObjectPoses(std::vector<std::string>(1, obj_name));
      for (std::pair<std::string, geometry_msgs::Pose> ao_pose: ao_poses)
      {
        ROS_ERROR_STREAM("AFTER ATTACH    " << ao_pose.first << " object at " << pose2str3D(ao_pose.second) << "        "<< arm_.getEndEffectorLink() << "   "<<arm_.getEndEffector());
      }


      if  (place(obj_name, place_pose))
      {
        std::map<std::string, geometry_msgs::Pose> ao_poses = planning_scene_interface_.getObjectPoses(std::vector<std::string>(1, obj_name));
        for (std::pair<std::string, geometry_msgs::Pose> ao_pose: ao_poses)
        {
          ROS_ERROR_STREAM("AFTER PLACE    " << ao_pose.first << " object at " << pose2str3D(ao_pose.second) << "        "<< arm_.getEndEffectorLink() << "   "<<arm_.getEndEffector());
        }
        as_.setSucceeded(result_);
        return true;
      }
    }
//    else
//    {
//      std::map<std::string, geometry_msgs::Pose> ao_poses = planning_scene_interface_.getObjectPoses(std::vector<std::string>(1, obj_name));
//      for (std::pair<std::string, geometry_msgs::Pose> ao_pose: ao_poses)
//      {
//        ROS_ERROR_STREAM(ao_pose.first << " object at " << pose2str3D(ao_pose.second));
//      }
      as_.setAborted(result_);
      return false;
//    }
  }

  bool pick(const std::string& obj_name, const geometry_msgs::Pose& pose)
  {
    ROS_INFO("[pick and place] Picking...");

    for (int attempt = 0; attempt < PICK_ATTEMPTS; ++attempt)
    {
      geometry_msgs::PoseStamped p;
      p.pose = pose;

      if (!validateTargetPose(p, attempt))
      {
        return false;
      }
///// TODO
//        # Set and id for this grasp (simply needs to be uniq#ue)
//        g.id = str(len(grasps))
//
//        # Set the allowed touch objects to the input list
//        g.allowed_touch_objects = allowed_touch_objects
//
//        # Don't restrict contact force
//        g.max_contact_force = 0
//
//        # Degrade grasp quality for increasing pitch angles
//        g.grasp_quality = 1.0 - abs(pitch)
//
//        # Append the grasp to the list
//        grasps.append(deepcopy(g))

      std::vector<moveit_msgs::Grasp> grasps;
      moveit_msgs::Grasp g;
      g.grasp_pose = p;

      g.pre_grasp_approach.direction.vector.x = 0.5;
      //g.pre_grasp_approach.direction.vector.z = -0.5;
      g.pre_grasp_approach.direction.header.frame_id = arm_.getEndEffectorLink();
      g.pre_grasp_approach.min_distance = 0.005;
      g.pre_grasp_approach.desired_distance = 0.1;

      g.post_grasp_retreat.direction.header.frame_id = arm_.getEndEffectorLink();
      g.post_grasp_retreat.direction.vector.x = -0.5;
//      g.post_grasp_retreat.direction.vector.z = 0.5;
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

      grasps.push_back(g);

      if (arm_.pick(obj_name, grasps))
      {
        ROS_INFO("[pick and place] Pick successfully completed");

        ros::Duration(2.0).sleep();
        return true;
      }
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }

    ROS_ERROR("[pick and place] Pick failed after %d attempts", PICK_ATTEMPTS);
    return false;
  }

  bool place(const std::string& obj_name, const geometry_msgs::Pose& pose)
  {
    ROS_INFO("[pick and place] Placing...");

    // place
    for (int attempt = 0; attempt < PLACE_ATTEMPTS; ++attempt)
    {
      geometry_msgs::PoseStamped p;
      p.pose = pose;

      if (!validateTargetPose(p, attempt))
      {
        return false;
      }

      std::vector<moveit_msgs::PlaceLocation> loc;
      moveit_msgs::PlaceLocation l;
      l.place_pose = p;

      l.pre_place_approach.direction.vector.x = 0.5;
//      l.pre_place_approach.direction.vector.z = -0.1;
      l.pre_place_approach.direction.header.frame_id = arm_.getEndEffectorLink();
      l.pre_place_approach.min_distance = 0.005;
      l.pre_place_approach.desired_distance = 0.1;

      l.post_place_retreat.direction.vector.x = -0.5;
//      l.post_place_retreat.direction.vector.z = 0.1;
      l.post_place_retreat.direction.header.frame_id = arm_.getEndEffectorLink();
      l.post_place_retreat.min_distance = 0.005;
      l.post_place_retreat.desired_distance = 0.1;

      l.post_place_posture.joint_names.resize(1, "gripper_joint");
      l.post_place_posture.points.resize(1);
      l.post_place_posture.points[0].positions.resize(1);
      l.post_place_posture.points[0].positions[0] = gripper_open;

      l.allowed_touch_objects.push_back("table");

      loc.push_back(l);

      // add path constraints
//      moveit_msgs::Constraints constr;
//      constr.orientation_constraints.resize(1);
//      moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
//      ocm.link_name = "r_wrist_roll_link";
//      ocm.header.frame_id = p.header.frame_id;
//      ocm.orientation.x = 0.0;
//      ocm.orientation.y = 0.0;
//      ocm.orientation.z = 0.0;
//      ocm.orientation.w = 1.0;
//      ocm.absolute_x_axis_tolerance = 0.2;
//      ocm.absolute_y_axis_tolerance = 0.2;
//      ocm.absolute_z_axis_tolerance = M_PI;
//      ocm.weight = 1.0;
      //  group.setPathConstraints(constr);
//      group.setPlannerId("RRTConnectkConfigDefault");


      if (arm_.place(obj_name, loc))
      {
        ROS_INFO("[pick and place] Place successfully completed");
        return true;
      }
      ros::Duration(2.0).sleep();
      ros::spinOnce();
    }

    ROS_ERROR("[pick and place] Place failed after %d attempts", PLACE_ATTEMPTS);
    return false;
  }


private:

  /**
   * Move arm to a target pose. Only position coordinates are taken into account; the
   * orientation is calculated according to the direction and distance to the target.
   * @param target Pose target to achieve
   * @return True of success, false otherwise
   */
  bool validateTargetPose(geometry_msgs::PoseStamped& target, int attempt = 0)
  {
    // Pitch angles to try
    // pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.4, -0.4]

    // Yaw angles to try; given the limited dofs of turtlebot_arm, we must calculate the heading
    // from arm base to the object to pick (first we must transform its pose to arm base frame)
    // target_pose_arm_ref = self.tf_listener.transformPose(ARM_BASE_FRAME, initial_pose_stamped)
//      double x = p.pose.position.x;
//      double y = p.pose.position.y;
//
//      double yaw = atan2(y, x);   // check in make_places method why we store the calculated yaw
//      p.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 1.0, atan2(y, x));
//yaw_vals = [0, 0.1,-0.1, self.pick_yaw]
//
//# Generate a grasp for each pitch and yaw angle
//for yaw in yaw_vals:
//    for pitch in pitch_vals:
//        # Create a quaternion from the Euler angles
//        q = quaternion_from_euler(0, pitch, yaw)
//
//        # Set the grasp pose orientation accordingly
//        g.grasp_pose.pose.orientation.x = q[0]
//        g.grasp_pose.pose.orientation.y = q[1]
//        g.grasp_pose.pose.orientation.z = q[2]
//        g.grasp_pose.pose.orientation.w = q[3]

    target.header.frame_id = arm_link;

    double x = target.pose.position.x;
    double y = target.pose.position.y;
    double z = target.pose.position.z;
    double d = sqrt(x*x + y*y);
    if (d > 0.3)
    {
      // Maximum reachable distance by the turtlebot arm is 30 cm
      ROS_ERROR("[pick and place] Target pose out of reach [%f > %f]", d, 0.3);
      return false;
    }
    // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
    // we point the gripper. Yaw is the direction to the target. We also try some random variations of both to
    // increase the chances of successful planning.

    // Pitch is 90 (vertical) at 10 cm from the arm base; the farther the target is, the closer to horizontal
    // we point the gripper (0.205 = arm's max reach - vertical pitch distance + ε). Yaw is the direction to
    // the target. We also try some random variations of both to increase the chances of successful planning.
    // Roll is inverted with negative yaws so the arms doesn't make full turns when acting in different halfs
    // of the working space.
    // XXX: We don't need to try different values of yaw as long as this issue is implemented (or hacked) :
    //      https://github.com/ros-planning/moveit_ros/issues/577
    double rp = M_PI_2 - std::asin((d - 0.1)/0.205) + ((attempt%2)*2 - 1)*(std::ceil(attempt/2.0)*0.05);
    double ry = std::atan2(y, x);
    double rr = ry < 0.0 ? M_PI : 0.0;
    target.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rr, rp, ry);

    // Slightly increase z proportionally to pitch to avoid hitting the table with the lower gripper corner and
    // a bit extra to compensate the effect of the arm's backlash in the height of the gripper over the table
    double z_delta1 = std::abs(std::cos(rp))/50.0;
    double z_delta2 = z_backlash;
    ROS_DEBUG("[pick and place] Z increase:  %f  +  %f  +  %f", target.pose.position.z, z_delta1, z_delta2);
    target.pose.position.z += z_delta1;
    target.pose.position.z += z_delta2;

    ROS_DEBUG("[pick and place] Set pose target [%.2f, %.2f, %.2f] [d: %.2f, r: %.2f, p: %.2f, y: %.2f]",
              x, y, z, d, rr, rp, ry);
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




class MoveToTargetServer
{
private:

  ros::NodeHandle nh_;
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
  MoveToTargetServer(const std::string name,
                     const moveit::planning_interface::MoveGroup& arm,
                     const moveit::planning_interface::MoveGroup& gripper) :
    nh_("~"), as_(name, false), action_name_(name), arm_(arm), gripper_(gripper)
  {
    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&MoveToTargetServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveToTargetServer::preemptCB, this));

    as_.start();

    target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);
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

//    arm_.setNamedTarget(goal->named_target);
//    arm_.setSupportSurfaceName("table");
//
//    // Allow some leeway in position (meters) and orientation (radians)
//    arm_.setGoalPositionTolerance(0.001);
//    arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
//    arm_.allowReplanning(true);
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

  // Setup an asynchronous spinner as the move groups operations need continuous spinning
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroup arm("arm");
  moveit::planning_interface::MoveGroup gripper("gripper");

  turtlebot_arm_object_manipulation::PickAndPlaceServer pnp_server("pick_and_place", arm, gripper);
  turtlebot_arm_object_manipulation::MoveToTargetServer mtt_server("move_to_target", arm, gripper);
  ros::spin();

  spinner.stop();
  return 0;
}
