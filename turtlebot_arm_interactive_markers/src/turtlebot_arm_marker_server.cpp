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
#include <simple_arm_server/MoveArmAction.h>
#include <arbotix_msgs/Relax.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string.h>

#include <boost/foreach.hpp>

using namespace visualization_msgs;
using namespace interactive_markers;
using namespace std;

class TurtlebotArmMarkerServer
{
  private:
    ros::NodeHandle nh;
    actionlib::SimpleActionClient<simple_arm_server::MoveArmAction> client;
    interactive_markers::InteractiveMarkerServer server;
    tf::TransformListener tf_listener;
    
    MenuHandler arm_menu_handler;
    map<std::string, MenuHandler> joint_menu_handlers;
    
    bool immediate_commands;
    bool in_move;
    
    ros::Timer arm_timer;
    
    // Joint and link parameters
    vector<string> joints;
    vector<string> links;
    string tip_link;
    string root_link;
    
    // Gripper parameters
    double gripper_marker_offset_x;
    double gripper_marker_offset_y;
    double gripper_marker_offset_z;
    
    double gripper_box_offset_x;
    double gripper_box_offset_y;
    double gripper_box_offset_z;
    
    
    // Joint command and relax publishers
    map<std::string, ros::Publisher> joint_command_publishers;
    map<std::string, ros::ServiceClient> joint_relax_clients;
    
public:
  TurtlebotArmMarkerServer()
    : nh("~"), client("move_arm", true), server("turtlebot_arm_marker_server"), tf_listener(nh), immediate_commands(true), in_move(false)
  {
  
    ROS_INFO("[turtlebot arm marker server] Started initializing.");
    
    std::string arm_server_topic;
    
    // Get general arm parameters
    nh.param<std::string>("arm_server_topic", arm_server_topic, "/simple_arm_server/move");
    nh.param<std::string>("root_link", root_link, "/arm_base_link");
    nh.param<std::string>("tip_link", tip_link, "/gripper_link");
    
    // Get the joint list
    XmlRpc::XmlRpcValue joint_list;
    nh.getParam("joints", joint_list);
    ROS_ASSERT(joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_list.size(); ++i) 
    {
      ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      joints.push_back(joint_list[i]);
    }
    
    // Get the corresponding link list
    XmlRpc::XmlRpcValue link_list;
    nh.getParam("links", link_list);
    ROS_ASSERT(link_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_list.size(); ++i) 
    {
      ROS_ASSERT(link_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      links.push_back(link_list[i]);
    }
    
    // Get gripper offsets
    nh.param<double>("gripper/marker_offset/x", gripper_marker_offset_x, 0.02);
    nh.param<double>("gripper/marker_offset/y", gripper_marker_offset_y, 0.025);
    nh.param<double>("gripper/marker_offset/z", gripper_marker_offset_z, 0.0);
    
    nh.param<double>("gripper/box_offset/x", gripper_box_offset_x, -0.017);
    nh.param<double>("gripper/box_offset/y", gripper_box_offset_y, -0.008);
    nh.param<double>("gripper/box_offset/z", gripper_box_offset_z, 0.0);
    
    
    ROS_INFO("[turtlebot arm marker server] Got parameters.");
    
    /*
    joints.push_back("shoulder_pan_joint");
    links.push_back("dynamixel_AX12_0_link");
    
    joints.push_back("shoulder_lift_joint");
    links.push_back("dynamixel_AX12_1_link");
    
    joints.push_back("elbow_flex_joint");
    links.push_back("dynamixel_AX12_2_link");
    
    joints.push_back("wrist_flex_joint");
    links.push_back("dynamixel_AX12_3_link");
    
    joints.push_back("gripper_joint");
    links.push_back("dynamixel_AX12_4_link"); */
    
    //client = nh.serviceClient<simple_arm_server::MoveArm>(arm_server_topic);
    //client.waitForServer();

    createArmMarker();
    createGripperMarker();
    createArmMenu();
    
    createJointPublishers();
    
    resetMarker();
    
    ROS_INFO("[turtlebot arm marker server] Initialized.");
    
    //arm_timer = nh.createTimer(ros::Duration(0.1), &TurtlebotArmMarkerServer::resetMarkerCb, this);
  }

  void createJointPublishers()
  {
    BOOST_FOREACH( std::string joint_name, joints )
    {
      joint_command_publishers[joint_name] = (nh.advertise<std_msgs::Float64>("/" + joint_name + "/command", 1, false));
      joint_relax_clients[joint_name] = (nh.serviceClient<arbotix_msgs::Relax>("/" + joint_name + "/relax"));
    }
  
  }

  void processArmFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN && feedback->marker_name == "arm_marker")
    {
      arm_timer.stop();
    }
    if (feedback->event_type != visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP || feedback->marker_name != "arm_marker")
      return;
      
    if (immediate_commands)
    {
      processCommand(feedback);
    }
  }
  
  void processJointFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std_msgs::Float64 command;
    command.data = feedback->pose.orientation.y;
    joint_command_publishers[feedback->marker_name].publish(command);
  }
  
  void changeMarkerColor(double r, double g, double b, bool set_pose=false, geometry_msgs::Pose pose = geometry_msgs::Pose())
  {
    InteractiveMarker int_marker;
    server.get("arm_marker", int_marker);
    
    Marker *box_marker = &int_marker.controls[0].markers[0];
    
    box_marker->color.r = r;
    box_marker->color.g = g;
    box_marker->color.b = b;
    
    if (set_pose)
      int_marker.pose = pose;
    
    server.insert(int_marker);
    server.applyChanges();
  }
  
  void processCommand(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    //changeMarkerColor(0, 0, 1);
    if (sendTrajectoryCommand(feedback))
    {
      changeMarkerColor(0, 1, 0, true, feedback->pose);
      ros::Duration(1.0).sleep();
      resetMarker();
      //arm_timer.start();
    }
    else
    {
      changeMarkerColor(1, 0, 0, true, feedback->pose);
    }
  }
  
  bool sendTrajectoryCommand(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    simple_arm_server::MoveArmGoal goal;
    simple_arm_server::ArmAction action;
    
    goal.header.frame_id = root_link;
    
    geometry_msgs::Pose pose;
    getTransformedPose(feedback->header.frame_id, feedback->pose, root_link, pose, feedback->header.stamp);
    
    action.goal.orientation = pose.orientation;
    action.goal.position = pose.position;
    action.move_time = ros::Duration(1.0);
    goal.motions.push_back(action); 
    
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(30.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    return false;
  }
  
  bool sendGripperCommand(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    simple_arm_server::MoveArmGoal goal;
    simple_arm_server::ArmAction action;
    
    goal.header.frame_id = tip_link;
    action.type = simple_arm_server::ArmAction::MOVE_GRIPPER;
    action.command = feedback->pose.position.y;
    goal.motions.push_back(action); 
    
    client.sendGoal(goal);
    client.waitForResult(ros::Duration(30.0));
    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      return true;
    return false;
  }

  void getTransformedPose(const string& source_frame, const geometry_msgs::Pose& source_pose,
                          const string& target_frame, geometry_msgs::Pose& target_pose,
                          const ros::Time& time)
  {
    tf::Pose bt_source_pose;
    
    tf::poseMsgToTF(source_pose, bt_source_pose);
    
    tf::Stamped<tf::Pose> posein(bt_source_pose, time, source_frame);
    tf::Stamped<tf::Pose> poseout;
    
    try {
        ros::Duration timeout(10.0);
        
        // Get base_link transform
        tf_listener.waitForTransform(target_frame, source_frame,
                                      time, timeout);
        tf_listener.transformPose(target_frame, posein, poseout);
        
        
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[arm interactive markers] TF exception:\n%s", ex.what());
        return;
      }
      
    tf::poseTFToMsg(poseout, target_pose);
  }

  void resetMarker()
  {
    geometry_msgs::Pose arm_pose;
    geometry_msgs::Pose blank_pose;
    blank_pose.orientation.w = 1;
    
    getTransformedPose(tip_link, blank_pose, root_link, arm_pose, ros::Time(0));
    
    server.setPose("arm_marker", arm_pose);
    server.applyChanges();
  }

  void resetMarkerCb(const ros::TimerEvent& e)
  {
    //resetMarker();
  }

  void createArmMarker()
  { 
    // Create a marker representing the tip of the arm.
    InteractiveMarker int_marker;
    int_marker.header.frame_id = root_link;
    int_marker.name = "arm_marker";
    int_marker.scale = 0.1;

    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = .7;

    InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.interaction_mode = InteractiveMarkerControl::BUTTON;
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

    
    server.insert(int_marker, boost::bind( &TurtlebotArmMarkerServer::processArmFeedback, this, _1 ));
    
    server.applyChanges();
  }
   
  void createGripperMarker()
  {
    // Create a marker for the gripper
    InteractiveMarker int_marker;
    int_marker.header.frame_id = tip_link;
    int_marker.name = "gripper_marker";
    int_marker.scale = 0.075;

    int_marker.pose.position.x = gripper_marker_offset_x;
    int_marker.pose.position.y = gripper_marker_offset_y;
    int_marker.pose.position.z = gripper_marker_offset_z;

    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = 0.05;
    marker.scale.y = 0.005;
    marker.scale.z = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = .7;
    
    marker.pose.position.x = gripper_box_offset_x;
    marker.pose.position.y = gripper_box_offset_y;
    marker.pose.position.z = gripper_box_offset_z;

    InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.interaction_mode = InteractiveMarkerControl::BUTTON;
    box_control.markers.push_back( marker );
    int_marker.controls.push_back( box_control );

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server.insert(int_marker, boost::bind( &TurtlebotArmMarkerServer::sendGripperCommand, this, _1 ));
    
    server.applyChanges();
  }
  
  void createJointMarkers()
  {
    for (unsigned int i = 0; i < joints.size() && i < links.size(); i++)
    {
      createJointMarker(joints[i], links[i]);
    }
  }
  
  void createJointMarker(const string joint_name, const string link_name)
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = link_name;
    int_marker.name = joint_name;
    int_marker.scale = 0.05;

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    server.insert(int_marker, boost::bind( &TurtlebotArmMarkerServer::processJointFeedback, this, _1 ));
    
    server.applyChanges();
  }
  
  void createArmMenu()
  {
    arm_menu_handler = MenuHandler();
    arm_menu_handler.insert("Send command", boost::bind(&TurtlebotArmMarkerServer::sendCommandCb, this, _1));
    arm_menu_handler.setCheckState(
        arm_menu_handler.insert("Send command immediately", boost::bind(&TurtlebotArmMarkerServer::immediateCb, this, _1)), MenuHandler::CHECKED );
    arm_menu_handler.insert("Reset marker", boost::bind(&TurtlebotArmMarkerServer::resetPoseCb, this, _1));
    
    MenuHandler::EntryHandle entry = arm_menu_handler.insert("Relax joints");
    arm_menu_handler.insert( entry, "All", boost::bind(&TurtlebotArmMarkerServer::relaxAllCb, this, _1));
    BOOST_FOREACH(string joint_name, joints)
    {
      arm_menu_handler.insert( entry, joint_name, boost::bind(&TurtlebotArmMarkerServer::relaxCb, this, joint_name, _1) );
    }
    
    arm_menu_handler.insert("Switch to Joint Control", boost::bind(&TurtlebotArmMarkerServer::switchToJointControlCb, this, _1));
    
    arm_menu_handler.apply( server, "arm_marker" );
    server.applyChanges();
  }
  
  void createJointMenus()
  {
    BOOST_FOREACH(string joint_name, joints)
    {
      createJointMenu(joint_name);
    }
  }
  
  void createJointMenu(const string joint_name)
  {
    MenuHandler joint_handler;
    joint_handler.insert("Relax all", boost::bind(&TurtlebotArmMarkerServer::relaxAllCb, this, _1));
    joint_handler.insert("Relax this joint", boost::bind(&TurtlebotArmMarkerServer::relaxCb, this, joint_name, _1));
    joint_handler.insert("Switch to Trajectory Control", boost::bind(&TurtlebotArmMarkerServer::switchToArmControlCb, this, _1));
    
    joint_menu_handlers[joint_name] = joint_handler;
    
    joint_menu_handlers[joint_name].apply( server, joint_name );
    server.applyChanges();
  }

  void sendCommandCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    processCommand(feedback);
  }
  
  void relaxAllCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    BOOST_FOREACH( std::string joint_name, joints )
    {
      relaxCb(joint_name, feedback);
    }
  }
  
  void relaxCb(const std::string joint_name, const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    arbotix_msgs::Relax srv;
    joint_relax_clients[joint_name].call(srv);
  }
  
  void immediateCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    MenuHandler::EntryHandle handle = feedback->menu_entry_id;
    MenuHandler::CheckState state;
    arm_menu_handler.getCheckState( handle, state );

    if ( state == MenuHandler::CHECKED )
    {
      arm_menu_handler.setCheckState( handle, MenuHandler::UNCHECKED );
      ROS_INFO("Switching to cached commands");
      immediate_commands = false;
    }
    else
    {
      arm_menu_handler.setCheckState( handle, MenuHandler::CHECKED );
      ROS_INFO("Switching to immediate comands");
      immediate_commands = true;
    }
    arm_menu_handler.reApply(server);
    
    server.applyChanges();
  }
  
  void resetPoseCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    resetMarker();
    arm_timer.start();
  }
  
  void switchToJointControlCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    server.clear();
    server.applyChanges();
    
    createJointMarkers();
    createJointMenus();
  }
  
  void switchToArmControlCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    server.clear();
    server.applyChanges();
    
    createArmMarker();
    createGripperMarker();
    //createArmMenu();
    
    arm_menu_handler.reApply(server);
    
    resetMarker();
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_arm_marker_server");
  TurtlebotArmMarkerServer turtleserver;
  
  ros::spin();
}
