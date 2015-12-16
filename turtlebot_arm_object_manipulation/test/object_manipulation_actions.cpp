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
#include <turtlebot_arm_object_manipulation/ObjectDetectionAction.h>
#include <turtlebot_arm_object_manipulation/PickAndPlaceAction.h>

#include <interactive_markers/interactive_marker_server.h>


using namespace visualization_msgs;

const std::string arm_link = "/arm_base_link";


class ObjectManipulationAction
{
private:
  interactive_markers::InteractiveMarkerServer server_;
  
  ros::NodeHandle nh_;
  
  // Actions
  actionlib::SimpleActionClient<turtlebot_arm_object_manipulation::ObjectDetectionAction> object_detection_action_;
  actionlib::SimpleActionClient<turtlebot_arm_object_manipulation::PickAndPlaceAction> pick_and_place_action_;
  
  turtlebot_arm_object_manipulation::ObjectDetectionGoal object_detection_goal_;
  turtlebot_arm_object_manipulation::PickAndPlaceGoal pick_and_place_goal_;
  
  geometry_msgs::Pose old_pose_;

public:

  ObjectManipulationAction() :
    server_("object_controls"),
    object_detection_action_("object_detection", true),
    pick_and_place_action_("pick_and_place", true)
  {
    server_.applyChanges();
    
    // Initialize goals
    object_detection_goal_.frame = arm_link;
    
    pick_and_place_goal_.frame = arm_link;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    object_detection_action_.waitForServer();
    pick_and_place_action_.waitForServer();
    
    ROS_INFO("Found servers.");
    
    detectObjects();
  }
  
  void detectObjects()
  {
    server_.clear();
    server_.applyChanges();
    
    object_detection_action_.sendGoal(object_detection_goal_,  boost::bind( &ObjectManipulationAction::addObjects, this, _1, _2));
  }
  
  void addObjects(const actionlib::SimpleClientGoalState& state, const turtlebot_arm_object_manipulation::ObjectDetectionResultConstPtr& result)
  {
    ROS_INFO("Got object detection callback. Adding objects.");
    
    for (unsigned int i=0; i < result->obj_names.size(); i++)
    {
      // addObject(object.position.x, object.position.y, object.position.z, i);  TODO
      ROS_INFO("Added %d objects: '%s'", i, result->obj_names[i].c_str());
    }

    server_.applyChanges(); 
  }
  

  // Move the real object!
  void feedbackCb( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM("Staging " << feedback->marker_name);     
          old_pose_ = feedback->pose;
        break;
   
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM("Now moving " << feedback->marker_name); 
        moveObject(old_pose_, feedback->pose);
        detectObjects();
        break;
    }
    
    server_.applyChanges(); 
  }
  
  bool moveObject(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    pick_and_place_goal_.pick_pose = start_pose;
    pick_and_place_goal_.place_pose = end_pose;
    pick_and_place_action_.sendGoalAndWait(pick_and_place_goal_, ros::Duration(30.0), ros::Duration(30.0));
    
    if ( object_detection_action_.getState()  == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // Make a box
  Marker makeBox( InteractiveMarker &msg, float r, float g, float b )
  {
    Marker m;

    m.type = Marker::CUBE;
    m.scale.x = msg.scale;
    m.scale.y = msg.scale;
    m.scale.z = msg.scale;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0;

    return m;
  }
   
  // Add a new object
  void addObject( float x, float y, float z, int n)
  {
    InteractiveMarker marker;
    marker.header.frame_id = arm_link;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale = 0.03;
    
    std::stringstream conv;
    conv << n;
    conv.str();
     
    marker.name = "object" + conv.str();

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    marker.controls.push_back( control );
    
    control.markers.push_back( makeBox(marker, .5, .5, .5) );
    control.always_visible = true;
    marker.controls.push_back( control );
    

    server_.insert( marker );
    server_.setCallback( marker.name, boost::bind( &ObjectManipulationAction::feedbackCb, this, _1 ));
  }

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "object_manipulation");

  ObjectManipulationAction manip;

  // everything is done in cloud callback, just spin
  ros::spin();
}

