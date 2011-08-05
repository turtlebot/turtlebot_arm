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
#include <simple_arm_server/MoveArm.h>
#include <simple_arm_server/ArmAction.h>

#include <interactive_markers/interactive_marker_server.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>


#include <tf/tf.h>

#include <string.h>

using namespace visualization_msgs;
using namespace std;

class TurtlebotArmMarkerServer
{
  public:
    TurtlebotArmMarkerServer()
      : nh("~"), server("turtlebot_arm_marker_server")
    {
      std::string arm_server_topic;
      
      nh.param<std::string>("arm_server_topic", arm_server_topic, "/simple_arm_server/move");
      //nh.param<double>("linear_scale", linear_scale, 1.0);
      //nh.param<double>("angular_scale", angular_scale, 2.2);
      
      //vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
      client = nh.serviceClient<simple_arm_server::MoveArm>(arm_server_topic);
      createInteractiveMarkers();
    }
    
    void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback );
    void resetMarker();
  
  private:
    void createInteractiveMarkers();
  
    ros::NodeHandle nh;
    ros::ServiceClient client;
    interactive_markers::InteractiveMarkerServer server;
    
    double linear_scale;
    double angular_scale;
};

void TurtlebotArmMarkerServer::processFeedback(
    const InteractiveMarkerFeedbackConstPtr &feedback )
{
  if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    return;
  // Alright let's move the arm or something
  simple_arm_server::MoveArm srv;
  simple_arm_server::ArmAction action;
  
  srv.request.header.frame_id="/dynamixel_AX12_4_link";
  
  // Eh, could do this in one go but whatever. This seems good.
  action.goal.orientation = feedback->pose.orientation;
  action.goal.position = feedback->pose.position;
  action.move_time = ros::Duration(1.0);
  
  srv.request.goals.push_back(action); 
  
  double success = false;
  if (client.call(srv))
  {
    success = srv.response.success;
    cout << "Success? " << success << endl;
  }
  else
  {
    cout << "Failure? " << success << endl;
  }
  
  resetMarker();
}

void TurtlebotArmMarkerServer::resetMarker()
{
  // Yeeees, except we actually have to do this properly or something.
  // Like... use tf to figure out the pose of the arm.
  // Yeeeah.
  // Maybe not if we just use the frame of the last joint!
  server.setPose("arm_marker", geometry_msgs::Pose());
  
  server.applyChanges();
}

void TurtlebotArmMarkerServer::createInteractiveMarkers()
{ 
  // create an interactive marker for our server
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "/dynamixel_AX12_4_link";
  int_marker.name = "arm_marker";
  int_marker.scale = 0.2;

  
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( marker );
  int_marker.controls.push_back( box_control );

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  
  server.insert(int_marker, boost::bind( &TurtlebotArmMarkerServer::processFeedback, this, _1 ));
  
  server.applyChanges();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_arm_marker_server");
  TurtlebotArmMarkerServer turtleserver;
  
  ros::spin();
}
