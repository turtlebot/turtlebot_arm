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

#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// action client: ORK's tabletop object recognition
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_recognition_msgs/TableArray.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>

// action server: make things easier for interactive manipulation
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_arm_object_manipulation/ObjectDetectionAction.h>

// MoveIt!
#include <moveit_msgs/ObjectColor.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>


namespace turtlebot_arm_object_manipulation
{

class ObjectDetectionServer
{
private:

  // ROS interface
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Action client for the ORK object recognition and server
  actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ork_ac_;

  // Action server to handle it conveniently for our object manipulation demo
  actionlib::SimpleActionServer<turtlebot_arm_object_manipulation::ObjectDetectionAction> od_as_;
  turtlebot_arm_object_manipulation::ObjectDetectionFeedback feedback_;
  turtlebot_arm_object_manipulation::ObjectDetectionResult result_;
  turtlebot_arm_object_manipulation::ObjectDetectionGoalConstPtr goal_;
  std::string action_name_;

  // Get object information from database service
  ros::ServiceClient obj_info_srv_;

  // Publishers and subscribers
  ros::Subscriber table_sub_;

  ros::Publisher block_pub_;
  ros::Publisher c_obj_pub_;
  ros::Publisher scene_pub_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf::TransformListener tf_listener_;

  // Parameters from goal
  std::string arm_link_;
  double obj_size_;
  
public:
  ObjectDetectionServer(const std::string name) :
    pnh_("~"), ork_ac_("tabletop/recognize_objects", true), od_as_(name, false), action_name_(name)
  {
    // create the action client; spin its own thread

    // wait for the tabletop/recognize_objects action server to start before we provide our own service
    ROS_INFO("[object detection] Waiting for tabletop/recognize_objects action server to start...");
    ork_ac_.waitForServer();

    ROS_INFO("[object detection] tabletop/recognize_objects action server started; ready for sending goals.");

    // wait for the get object information service
    // CANNOT FIND IT!!! XXX obj_info_srv_ = nh_.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    // TODO: I must do it by myself...  based on object_search.py script

    // Register the goal and feedback callbacks.
    od_as_.registerGoalCallback(boost::bind(&ObjectDetectionServer::goalCB, this));
    od_as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this));
    
    od_as_.start();
    
    // Subscribe to detected tables array
    table_sub_ = nh_.subscribe("tabletop/table_array", 1, &ObjectDetectionServer::tableCb, this);

    // Public detected blocks poses
    block_pub_ = nh_.advertise<geometry_msgs::PoseArray>("turtlebot_blocks", 1, true);

    // Public planning scene changes
    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[object detection] Received goal!");

    // accept the new goal
    goal_ = od_as_.acceptNewGoal();
    
    arm_link_ = goal_->frame;
    obj_size_ = goal_->obj_size;

    result_.obj_poses.poses.clear();
    result_.obj_names.clear();
    result_.obj_poses.header.frame_id = arm_link_;

    // send a goal to the action server
    ROS_INFO("[object detection] Sending goal to tabletop/recognize_objects action server.");
    object_recognition_msgs::ObjectRecognitionGoal goal;
    ork_ac_.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ork_ac_.waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ork_ac_.getState();
      ROS_INFO("[object detection] Action finished: %s",state.toString().c_str());
    }
    else
    {
      ROS_WARN("[object detection] Tabletop/recognize_objects action did not finish before the time out.");
      od_as_.setAborted(result_, "Tabletop/recognize_objects action did not finish before the time out");
      return;
    }

    object_recognition_msgs::ObjectRecognitionResultConstPtr result = ork_ac_.getResult();

    // Add all detected objects to the goal result; TODO do some filtering!!!
    for (object_recognition_msgs::RecognizedObject obj: result->recognized_objects.objects)
    {
      addBlock(obj);
    }

    if (result_.obj_poses.poses.size() > 0)
    {
      block_pub_.publish(result_.obj_poses);
      ROS_INFO("[object detection] Set as succeeded!");
    }
    else
    {
      ROS_INFO("[object detection] Couldn't find any blocks this iteration!");
    }

    od_as_.setSucceeded(result_);
  }

  void preemptCB()
  {
    ROS_WARN("[object detection] %s: Preempted", action_name_.c_str());
    // set the action state to preempted
    od_as_.setPreempted();
  }

  void tableCb(const object_recognition_msgs::TableArray& msg)
  {
    if (msg.tables.size() == 0)
    {
      ROS_WARN("[object detection] Table array message is empty");
      return;
    }

    // Add the table as a collision object, so it gets filtered out from MoveIt! octomap
    for (object_recognition_msgs::Table table: msg.tables)
    {
      addTable(table);
    }
  }


private:

  void addBlock(const object_recognition_msgs::RecognizedObject& obj)
  {
    if (obj.confidence < 0.9)
      return;

    std::string obj_name;
    std::stringstream sstream;
    sstream << "block_" << result_.obj_names.size() + 1;
    obj_name = sstream.str();

    geometry_msgs::Pose out_pose;
    transformPose(obj.header.frame_id, arm_link_, obj.pose.pose.pose, out_pose);
    ROS_INFO_STREAM("[object detection] Adding \"" << obj_name << "\" object at "
                                                   << out_pose.position.x << ", "
                                                   << out_pose.position.y << ", "
                                                   << out_pose.position.z);
    result_.obj_poses.poses.push_back(out_pose);
    result_.obj_names.push_back(obj_name);

    moveit_msgs::CollisionObject co;
    co.header = result_.obj_poses.header;
    co.id = obj_name;
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, co.id));

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = obj_size_;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = obj_size_;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = obj_size_;
    co.primitive_poses.resize(1);
    co.primitive_poses[0] = out_pose;

//    if (! transformPose(table.header.frame_id, arm_link_, table.pose, co.primitive_poses[0]))
//      return;
//
//    co.primitive_poses[0].position.x += table_size_x/3.0;
//    co.primitive_poses[0].position.z -= table_size_z/2.0;

    std::vector<moveit_msgs::CollisionObject> collision_objects(1, co);
    planning_scene_interface_.addCollisionObjects(collision_objects);

    moveit_msgs::ObjectColor oc;
    oc.id = co.id;
    oc.color = getRandColor(1.0);
    moveit_msgs::PlanningScene ps;
    ps.object_colors.push_back(oc);
    ps.is_diff = true;
    scene_pub_.publish(ps);
  }

  void addTable(const object_recognition_msgs::Table& table)
  {
    // Add the table as a collision object into the world, so it gets excluded from the collision map
    // As the blocks are small, they should also be excluded (assuming that padding_offset parameter on
    // octomap sensor configuration is equal or bigger than object size)
    // TODO: estimate table size (and maybe shape, later...)
    double table_size_x = 0.55;
    double table_size_y = 0.55;
    double table_size_z = 0.005;

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = arm_link_;

    co.id = "table";
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, co.id));

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = table_size_x;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = table_size_y;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = table_size_z;
    co.primitive_poses.resize(1);

    if (! transformPose(table.header.frame_id, arm_link_, table.pose, co.primitive_poses[0]))
      return;

    co.primitive_poses[0].position.x += table_size_x/3.0;
    co.primitive_poses[0].position.z -= table_size_z/2.0;

    // Tables often have orientations with all-nan values, but assertQuaternionValid lets them go!
    if (std::isnan(co.primitive_poses[0].orientation.x) ||
        std::isnan(co.primitive_poses[0].orientation.y) ||
        std::isnan(co.primitive_poses[0].orientation.z) ||
        std::isnan(co.primitive_poses[0].orientation.w))
    {
      ROS_WARN("[object detection] Table orientation has nan values; assuming horizontal");
      co.primitive_poses[0].orientation = tf::createQuaternionMsgFromYaw(0.0);
    }

    ROS_INFO_STREAM("[object detection] Adding a table at " << co.primitive_poses[0].position.x << ", "
                                                            << co.primitive_poses[0].position.y << ", "
                                                            << co.primitive_poses[0].position.z
                                                            << " as a collision object into the world");
    std::vector<moveit_msgs::CollisionObject> collision_objects(1, co);
    planning_scene_interface_.addCollisionObjects(collision_objects);
  }

  bool transformPose(const std::string& in_frame, const std::string& out_frame,
                     const geometry_msgs::Pose& in_pose, geometry_msgs::Pose& out_pose)
  {
    geometry_msgs::PoseStamped in_stamped;
    geometry_msgs::PoseStamped out_stamped;

    in_stamped.header.frame_id = in_frame;
    in_stamped.pose = in_pose;
    try
    {
      tf_listener_.waitForTransform(in_frame, out_frame, ros::Time(0.0), ros::Duration(1.0));
      tf_listener_.transformPose(out_frame, in_stamped, out_stamped);
      out_pose = out_stamped.pose;

//      // Some verifications...
//      // tables sometimes have orientations with all-nan values, but assertQuaternionValid lets them go!
//      tf::assertQuaternionValid(out_stamped.pose.orientation);
//      if (std::isnan(out_pose.orientation.x) ||
//          std::isnan(out_pose.orientation.y) ||
//          std::isnan(out_pose.orientation.z) ||
//          std::isnan(out_pose.orientation.w))
//        throw tf::InvalidArgument("Quaternion contains nan values");

      return true;
    }
    catch (tf::InvalidArgument& e)
    {
      ROS_ERROR("[object detection] Transformed pose has invalid orientation: %s", e.what());
      return false;
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR("[object detection] Could not get sensor to arm transform: %s", e.what());
      return false;
    }
  }

  std_msgs::ColorRGBA getRandColor(float alpha = 1.0)
  {
    std_msgs::ColorRGBA color;
    color.r = float(rand())/RAND_MAX;
    color.g = float(rand())/RAND_MAX;
    color.b = float(rand())/RAND_MAX;
    color.a = alpha;
    return color;
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_action_server");

  turtlebot_arm_object_manipulation::ObjectDetectionServer server("object_detection");
  ros::spin();

  return 0;
}
