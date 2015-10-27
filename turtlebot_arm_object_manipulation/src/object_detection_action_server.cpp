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
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>


namespace turtlebot_arm_object_manipulation
{

class ObjectDetectionServer
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ac_;
  actionlib::SimpleActionServer<turtlebot_arm_object_manipulation::ObjectDetectionAction> as_;
  std::string action_name_;
  turtlebot_arm_object_manipulation::ObjectDetectionFeedback feedback_;
  turtlebot_arm_object_manipulation::ObjectDetectionResult result_;
  turtlebot_arm_object_manipulation::ObjectDetectionGoalConstPtr goal_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf::TransformListener tf_listener_;
  
  ros::Publisher block_pub_;
  ros::Publisher c_obj_pub_;
  
  // Parameters from goal
  std::string arm_link_;
  
public:
  ObjectDetectionServer(const std::string name) :
    pnh_("~"), ac_("tabletop/recognize_objects", true), as_(name, false), action_name_(name)
  {
    // create the action client; spin its own thread

    // wait for the tabletop/recognize_objects action server to start before we provide our own service
    ROS_INFO("Waiting for tabletop/recognize_objects action server to start...");
    ac_.waitForServer();

    ROS_INFO("tabletop/recognize_objects action server started; ready for sending goals.");

    // Register the goal and feeback callbacks.
    as_.registerGoalCallback(boost::bind(&ObjectDetectionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this));
    
    as_.start();
    
    // Subscribe to table array
    sub_ = nh_.subscribe("tabletop/table_array", 1, &ObjectDetectionServer::tableCb, this);

    // Publish the filtered point cloud for debug purposes
    //pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Public detected blocks poses
    block_pub_ = nh_.advertise<geometry_msgs::PoseArray>("turtlebot_blocks", 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[block detection] Received goal!");

    // accept the new goal
    goal_ = as_.acceptNewGoal();
    
    arm_link_ = goal_->frame;

    result_.blocks.poses.clear();
    result_.blocks.header.frame_id = arm_link_;

    // send a goal to the action server
    ROS_INFO("Sending goal to tabletop/recognize_objects action server.");
    object_recognition_msgs::ObjectRecognitionGoal goal;
    ac_.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(10.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac_.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
      ROS_INFO("Tabletop/recognize_objects action did not finish before the time out.");
      as_.setAborted(result_, "Tabletop/recognize_objects action did not finish before the time out");
      return;
    }

    object_recognition_msgs::ObjectRecognitionResultConstPtr result = ac_.getResult();

    // Add all detected objects to the goal result; TODO do some filtering!!!
    for (object_recognition_msgs::RecognizedObject obj: result->recognized_objects.objects)
    {
      addBlock(obj);
    }

    if (result_.blocks.poses.size() > 0)
    {
      block_pub_.publish(result_.blocks);
      ROS_INFO("[block detection] Set as succeeded!");
    }
    else
    {
      ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
    }

    as_.setSucceeded(result_);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void tableCb(const object_recognition_msgs::TableArray& msg)
  {
    if (msg.tables.size() == 0)
    {
      ROS_WARN("Table array message is empty");
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
    geometry_msgs::Pose out_pose;
    transformPose(obj.header.frame_id, arm_link_, obj.pose.pose.pose, out_pose);
    ROS_INFO_STREAM("Adding a new object at " << out_pose.position.x << ", "
                                              << out_pose.position.y << ", "
                                              << out_pose.position.z);
    result_.blocks.poses.push_back(out_pose);
  }

  void addTable(const object_recognition_msgs::Table& table)
  {
    // Add the table as a collision object into the world, so it gets excluded from the collision map
    // As the blocks are small, they should also be excluded (assuming that padding_offset parameter on
    // octomap sensor configuration is equal or bigger than block size)
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
      ROS_WARN("Table orientation has nan values; assuming horizontal and perpendicular to the arm frame");
      co.primitive_poses[0].orientation = tf::createQuaternionMsgFromYaw(0.0);
    }

    ROS_INFO_STREAM("Adding a table at " << co.primitive_poses[0].position.x << ", "
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
      ROS_ERROR("Transformed pose has invalid orientation: %s", e.what());
      return false;
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR("Could not get sensor to arm transform: %s", e.what());
      return false;
    }
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
