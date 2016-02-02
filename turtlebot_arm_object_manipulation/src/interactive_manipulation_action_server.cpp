/*
 * Copyright (c) 2015, Jorge Santos
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
 * Author: Jorge Santos
 */

#include <ros/ros.h>

// interactive manipulation markers action server
#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>

#include <turtlebot_arm_object_manipulation/InteractiveManipAction.h>

// auxiliary libraries
#include <yocs_math_toolkit/common.hpp>
#include <geometric_shapes/shape_operations.h>

// MoveIt!
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


using namespace visualization_msgs;


namespace turtlebot_arm_object_manipulation
{

class InteractiveManipulationServer
{
private:
  ros::NodeHandle nh_;

  interactive_markers::InteractiveMarkerServer server_;
  
  actionlib::SimpleActionServer<turtlebot_arm_object_manipulation::InteractiveManipAction> as_;
  std::string action_name_;

  geometry_msgs::Pose old_pose_;
  
  turtlebot_arm_object_manipulation::InteractiveManipFeedback     feedback_;
  turtlebot_arm_object_manipulation::InteractiveManipResult       result_;
  turtlebot_arm_object_manipulation::InteractiveManipGoalConstPtr goal_;

  // We use the planning_scene_interface::PlanningSceneInterface to retrieve world objects
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

public:

  InteractiveManipulationServer(const std::string name) : 
     nh_("~"), server_("move_objects"), as_(name, false), action_name_(name)
  {
    // Register the goal and feedback callbacks.
    as_.registerGoalCallback(boost::bind(&InteractiveManipulationServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&InteractiveManipulationServer::preemptCB, this));
    
    as_.start();
  }
  
  void goalCB()
  {
    // accept the new goal
    goal_ = as_.acceptNewGoal();
    
    ROS_INFO("[interactive manip] Received goal! Adding markers for objects in the word other than the table");
    addObjects();
  }

  void preemptCB()
  {
    ROS_WARN("[interactive manip] %s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }
  
  // Moving an object; keep MOUSE_DOWN pose (origin) and move the object to MOUSE_UP pose
  void feedbackCb(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (!as_.isActive())
    {
      ROS_INFO("[interactive manip] Got feedback but not active!");
      return;
    }
    switch (feedback->event_type)
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM("[interactive manip] Staging '" << feedback->marker_name << "' at " << feedback->pose);
        old_pose_ = feedback->pose;
        break;
   
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM("[interactive manip] Now moving '" << feedback->marker_name << "' to " << feedback->pose);
        moveObject(feedback->marker_name, feedback->header, old_pose_, feedback->pose);
        break;
    }
    
    server_.applyChanges(); 
  }
  
  void moveObject(const std::string& marker_name, const std_msgs::Header& poses_header,
                  const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    result_.header = poses_header;
    result_.obj_name = marker_name;
    result_.pick_pose = start_pose;
    result_.place_pose = end_pose;
    
    as_.setSucceeded(result_);
    
    server_.clear();
    server_.applyChanges();
  }

  // Add an interactive marker for any object in the word other than the table
  void addObjects()
  {
    server_.clear();
    server_.applyChanges();

    bool active = as_.isActive();

    std::map<std::string, moveit_msgs::CollisionObject> objects =
        planning_scene_interface_.getObjects(planning_scene_interface_.getKnownObjectNames());
    for (const std::pair<std::string, moveit_msgs::CollisionObject>& obj: objects)
    {
      if (obj.first != "table")
      {
        addMarker(obj.second, active);
      }
    }

    server_.applyChanges();
  }

  // Add an interactive marker for the given object
  bool addMarker(const moveit_msgs::CollisionObject& obj, bool active)
  {
    InteractiveMarker marker;
    marker.header = obj.header;
    marker.name = obj.id;

    // We get object's pose from the mesh/primitive poses; try first with the meshes
    if (obj.mesh_poses.size() > 0)
    {
      marker.pose = obj.mesh_poses[0];
      if (obj.meshes.size() > 0)
      {
        Eigen::Vector3d obj_size = shapes::computeShapeExtents(obj.meshes[0]);

        // We use the biggest dimension of the mesh to scale the marker
        marker.scale = obj_size.maxCoeff();

        // We assume meshes laying in the floor (well, table), so we bump up marker pose by half z-dimension
        marker.pose.position.z += obj_size[2]/2.0;
      }
      else
      {
        ROS_ERROR("[interactive manip] Collision object has no meshes");
        return false;
      }
    }
    else if (obj.primitive_poses.size() > 0)
    {
      marker.pose = obj.primitive_poses[0];
      if (obj.primitives.size() > 0)
      {
        // We use the biggest dimension of the primitive to scale the marker
        marker.scale = shapes::computeShapeExtents(obj.primitives[0]).maxCoeff();
      }
      else
      {
        ROS_ERROR("[interactive manip] Collision object has no primitives");
        return false;
      }
    }
    else
    {
      ROS_ERROR("[interactive manip] Collision object has no mesh/primitive poses");
      return false;
    }

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    
    if (active)
      marker.controls.push_back(control);
    
    control.markers.push_back(makeBox(marker, 0.5, 0.5, 0.5));
    control.markers.push_back(makeLabel(marker, 0.5, 0.5, 0.5));
    control.always_visible = true;
    marker.controls.push_back(control);
    
    server_.insert(marker);
    server_.setCallback(marker.name, boost::bind(&InteractiveManipulationServer::feedbackCb, this, _1));

    ROS_INFO("[interactive manip] Added interactive marker for object '%s' at [%s] and scale [%f]",
             marker.name.c_str(), mtk::pose2str3D(marker.pose).c_str(), marker.scale);

    return true;
  }

  // Make a box containing the object (5% bigger than the biggest dimension)
  Marker makeBox(InteractiveMarker &msg, float r, float g, float b)
  {
    Marker m;

    m.type = Marker::CUBE;
    m.scale.x = msg.scale + (msg.scale * 0.05);
    m.scale.y = msg.scale + (msg.scale * 0.05);
    m.scale.z = msg.scale + (msg.scale * 0.05);
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.1;

    return m;
  }

  // Make a label to show over the box
  Marker makeLabel(InteractiveMarker &msg, float r, float g, float b)
  {
    Marker m;

    m.type = Marker::TEXT_VIEW_FACING;
    m.text = msg.name;
    m.scale.x = 0.035;
    m.scale.y = 0.035;
    m.scale.z = 0.035;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.8;

    m.pose.position.z = msg.scale/2.0 + 0.025;

    return m;
  }

};

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "object_interactive_manip_action_server");

  turtlebot_arm_object_manipulation::InteractiveManipulationServer manip("interactive_manipulation");

  ros::spin();

  return 0;
}

