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

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

// action client: ORK's tabletop object recognition
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <object_recognition_msgs/TableArray.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <object_recognition_msgs/GetObjectInformation.h>
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
  class DetectionBin;  // forward declaration of private class DetectionBin

private:

  // ROS interface
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Action client for the ORK object recognition and server
  actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction> ork_ac_;
  ros::Duration ork_execute_timeout_;
  ros::Duration ork_preempt_timeout_;

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
  ros::Publisher clear_objs_pub_;
  ros::Publisher clear_table_pub_;

  std::vector<geometry_msgs::Pose> table_poses_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf::TransformListener tf_listener_;

  // Parameters from goal
  std::string arm_link_;
  double obj_size_;
  
  // Object detection and classification constants
  const double   CONFIDENCE_THRESHOLD  = 0.9;    // minimum confidence required to accept an object
  const double   CLUSTERING_THRESHOLD  = 0.05;   // maximum acceptable distance to assign an object to a bin
  const unsigned CALLS_TO_ORK_TABLETOP = 10;

public:
  ObjectDetectionServer(const std::string name) :
    pnh_("~"), ork_ac_("tabletop/recognize_objects", true), od_as_(name, false), action_name_(name),
    ork_execute_timeout_(5.0), ork_preempt_timeout_(1.0)
  {
    // Create the action client; spin its own thread

    // Wait for the tabletop/recognize_objects action server to start before we provide our own service
    ROS_INFO("[object detection] Waiting for tabletop/recognize_objects action server to start...");
    ork_ac_.waitForServer();

    ROS_INFO("[object detection] tabletop/recognize_objects action server started; ready for sending goals.");

    // Wait for the get object information service (optional)
    obj_info_srv_ = nh_.serviceClient<object_recognition_msgs::GetObjectInformation>("get_object_info");
    if (! obj_info_srv_.waitForExistence(ros::Duration(10.0)))
      ROS_WARN("Get object information service not available; we cannot provide objects readable name");

    // Register the goal and feedback callbacks.
    od_as_.registerGoalCallback(boost::bind(&ObjectDetectionServer::goalCB, this));
    od_as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this));
    
    od_as_.start();
    
    // Subscribe to detected tables array
    table_sub_ = nh_.subscribe("tabletop/table_array", 1, &ObjectDetectionServer::tableCb, this);

    // Publish detected blocks poses
    block_pub_ = nh_.advertise<geometry_msgs::PoseArray>("turtlebot_blocks", 1, true);

    // Publish planning scene changes
    scene_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);

    clear_objs_pub_ =
        nh_.advertise<object_recognition_msgs::RecognizedObjectArray>("/tabletop/recognized_object_array", 1, true);
    clear_table_pub_ =
        nh_.advertise<object_recognition_msgs::TableArray>("/tabletop/table_array", 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[object detection] Received goal!");

    // Accept the new goal
    goal_ = od_as_.acceptNewGoal();
    
    arm_link_ = goal_->frame;
    obj_size_ = goal_->obj_size;

    // Clear results from previous goals
    table_poses_.clear();
    result_.obj_poses.poses.clear();
    result_.obj_names.clear();
    result_.obj_poses.header.frame_id = arm_link_;

    planning_scene_interface_.removeCollisionObjects(planning_scene_interface_.getKnownObjectNames());


    // Call ORK tabletop action server and wait for the action to return
    // We do it CALLS_TO_TABLETOP times and accumulate the results on bins to provide more reliable results
    std::vector<DetectionBin> detection_bins;

    ROS_INFO("[object detection] Sending %d goals to tabletop/recognize_objects action server...",
             CALLS_TO_ORK_TABLETOP);
    object_recognition_msgs::ObjectRecognitionGoal goal;
    for (int i = 0; i < CALLS_TO_ORK_TABLETOP; ++i)
    {
      actionlib::SimpleClientGoalState state =
          ork_ac_.sendGoalAndWait(goal, ork_execute_timeout_, ork_preempt_timeout_);

      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("[object detection] Action successfully finished");
      }
      else
      {
        ROS_WARN("[object detection] Tabletop/recognize_objects action did not finish before the time out: %s",
                 state.toString().c_str());
        od_as_.setAborted(result_, "Tabletop/recognize_objects action did not finish before the time out");
        continue;
      }

      object_recognition_msgs::ObjectRecognitionResultConstPtr result = ork_ac_.getResult();

      // Classify objects detected in each call to tabletop into bins based on distance to bin's centroid
      for (const object_recognition_msgs::RecognizedObject& obj: result->recognized_objects.objects)
      {
        if (obj.confidence < CONFIDENCE_THRESHOLD)
          continue;

        bool assigned = false;
        for (DetectionBin& bin: detection_bins)
        {
          if (mtk::distance3D(bin.getCentroid().pose, obj.pose.pose.pose) <= CLUSTERING_THRESHOLD)
          {
            ROS_DEBUG("Object with pose [%s] added to bin %d with centroid [%s] with distance [%f]",
                      mtk::pose2str3D(obj.pose.pose.pose).c_str(), bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(),
                      mtk::distance3D(bin.getCentroid().pose, obj.pose.pose.pose));
            bin.addObject(obj);
            assigned = true;
            break;
          }
        }

        if (! assigned)
        {
          // No matching bin; create a new one for this object
          ROS_DEBUG("Object with pose [%s] added to a new bin", mtk::pose2str3D(obj.pose.pose.pose).c_str());
          DetectionBin new_bin;
          new_bin.id = detection_bins.size();
          new_bin.addObject(obj);
          detection_bins.push_back(new_bin);
        }
      }

      ros::spinOnce();  // keep spinning so table messages callbacks are processed
    }  // loop up to CALLS_TO_ORK_TABLETOP

    // Add a detected object per bin to the goal result, if the bin is consistent enough
    int added_objects = addObjects(detection_bins);
    if (added_objects > 0)
    {
      block_pub_.publish(result_.obj_poses);
      ROS_INFO("[object detection] Succeeded! %d objects detected", added_objects);

      // Add also the table as a collision object, so it gets filtered out from MoveIt! octomap
      if (table_poses_.size() > 0)
        addTable(table_poses_);
      else
        ROS_WARN("[object detection] No near-horizontal table detected!");

      // Clear recognized objects and tables from RViz by publishing empty messages, so they don't
      // mangle with interactive markers; shoddy... ORK visualizations should have expiration time
      object_recognition_msgs::RecognizedObjectArray roa;
      object_recognition_msgs::TableArray ta;

      roa.header.frame_id = arm_link_;
      ta.header.frame_id = arm_link_;

      clear_objs_pub_.publish(roa);
      clear_table_pub_.publish(ta);
    }
    else
    {
      ROS_INFO("[object detection] Succeeded, but couldn't find any blocks this iteration");
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

    // Accumulate table poses while detecting objects so the resulting pose (centroid) is more accurate
    for (const object_recognition_msgs::Table& table: msg.tables)
    {
      geometry_msgs::Pose table_pose = table.pose;
      // Tables often have orientations with all-nan values, but assertQuaternionValid lets them go!
      if (isnan(table.pose.orientation.x) ||
          isnan(table.pose.orientation.y) ||
          isnan(table.pose.orientation.z) ||
          isnan(table.pose.orientation.w))
      {
        ROS_WARN("[object detection] Table discarded as its orientation has nan values");
        continue;
      }

      if (! transformPose(msg.header.frame_id, arm_link_, table.pose, table_pose))
      {
        continue;
      }

      if ((std::abs(mtk::roll(table_pose)) < M_PI/10.0) && (std::abs(mtk::pitch(table_pose)) < M_PI/10.0))
      {
        // Only consider tables within +/-18 degrees away from the horizontal plane
        table_poses_.push_back(table_pose);
      }
      else
      {
        ROS_WARN("Table with pose [%s] discarded as it is %.2f radians away from the horizontal plane",
                 mtk::pose2str3D(table_pose).c_str(),
                 std::max(std::abs(mtk::roll(table_pose)), std::abs(mtk::pitch(table_pose))));
      }
    }
  }


private:

  int addObjects(const std::vector<DetectionBin>& detection_bins)
  {
    std::map<std::string, unsigned int> obj_name_occurences;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;

    // Add a detected object per bin to the goal result and to the planning scene as a collision object
    // Only bins receiving detections on most of the ORK tabletop calls are considered consistent enough
    for (const DetectionBin& bin: detection_bins)
    {
      if (bin.countObjects() < CALLS_TO_ORK_TABLETOP/1.5)
      {
        ROS_DEBUG("Bin %d with centroid [%s] discarded as it received %d objects out of %d attempts",
                   bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(), bin.countObjects(), CALLS_TO_ORK_TABLETOP);
        continue;
      }

      // Compose object name with the name provided by the database plus an index, starting with [1]
      std::string obj_name = getObjName(bin.getType());
      obj_name_occurences[obj_name]++;
      std::stringstream sstream;
      sstream << obj_name << " [" << obj_name_occurences[obj_name] << "]";
      obj_name = sstream.str();

      ROS_DEBUG("Bin %d with centroid [%s] and %d objects added as object %s",
                 bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(), bin.countObjects(), obj_name.c_str());

      geometry_msgs::Pose out_pose;
      transformPose(bin.getCentroid().header.frame_id, arm_link_, bin.getCentroid().pose, out_pose);
      ROS_INFO("[object detection] Adding \"%s\" object at %s", obj_name.c_str(), mtk::point2str(out_pose.position));
      result_.obj_poses.poses.push_back(out_pose);
      result_.obj_names.push_back(obj_name);

      moveit_msgs::CollisionObject co;
      co.header = result_.obj_poses.header;
      co.id = obj_name;

      co.operation = moveit_msgs::CollisionObject::ADD;
      co.primitives.resize(1);
      co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = obj_size_;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = obj_size_;
      co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = obj_size_;
      co.primitive_poses.resize(1);
      co.primitive_poses[0] = out_pose;

      collision_objects.push_back(co);

      // Provide a random color to the collision object
      moveit_msgs::ObjectColor oc;
      oc.id = co.id;
      oc.color = getRandColor(1.0);
      ps.object_colors.push_back(oc);
    }

    if (collision_objects.size() > 0)
    {
      planning_scene_interface_.addCollisionObjects(collision_objects);
      scene_pub_.publish(ps);
    }

    return collision_objects.size();
  }

  void addTable(const std::vector<geometry_msgs::Pose>& table_poses)
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

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = table_size_x;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = table_size_y;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = table_size_z;
    co.primitive_poses.resize(1);

    // Calculate table_pose as the centroid of all accumulated poses
    double roll_acc = 0.0, pitch_acc = 0.0, yaw_acc = 0.0;
    for (const geometry_msgs::Pose& pose: table_poses)
    {
      ROS_DEBUG("[object detection] Adding a table at %s based on %d observations",
                mtk::pose2str3D(pose).c_str(), table_poses.size());

      co.primitive_poses[0].position.x += pose.position.x;
      co.primitive_poses[0].position.y += pose.position.y;
      co.primitive_poses[0].position.z += pose.position.z;
      roll_acc                         += mtk::roll(pose);
      pitch_acc                        += mtk::pitch (pose);
      yaw_acc                          += mtk::yaw(pose);
    }

    co.primitive_poses[0].position.x /= (double)table_poses.size();
    co.primitive_poses[0].position.y /= (double)table_poses.size();
    co.primitive_poses[0].position.z /= (double)table_poses.size();
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_acc/(double)table_poses.size(),
                                                                                pitch_acc/(double)table_poses.size(),
                                                                                yaw_acc/(double)table_poses.size());
    // Displace the table center according to its harcoded size; TODO: remove once we estimate the table's size
    co.primitive_poses[0].position.x += table_size_x/3.0;
    co.primitive_poses[0].position.z -= table_size_z/2.0;

    ROS_INFO("[object detection] Adding a table at %s as a collision object into the world",
             mtk::point2str(co.primitive_poses[0].position));
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

  std::string getObjName(const object_recognition_msgs::ObjectType& obj_type)
  {
    // Get a human readable name from db using object's type
    if (obj_info_srv_.exists())
    {
      object_recognition_msgs::GetObjectInformation srv;
      srv.request.type = obj_type;
      if (obj_info_srv_.call(srv))
        return srv.response.information.name;

      ROS_ERROR("Call to object information service failed");
    }
    return obj_type.key;
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

  /**
   * Private class for clustering together the same object detected in consecutive observations
   * and provide a more accurate pose as the centroid of all observations.
   */
  class DetectionBin
  {
  public:
    unsigned int id;
    unsigned int countObjects() const { return objects.size(); }
    const geometry_msgs::PoseStamped& getCentroid() const { return centroid; }

    void addObject(object_recognition_msgs::RecognizedObject object)
    {
      objects.push_back(object);

      // recalculate centroid
      centroid = geometry_msgs::PoseStamped();
      double confidence_acc = 0.0, roll_acc = 0.0, pitch_acc = 0.0, yaw_acc = 0.0;
      for (const object_recognition_msgs::RecognizedObject& obj: objects)
      {
        centroid.header.stamp = ros::Time::now();
        centroid.header.frame_id = obj.pose.header.frame_id; // TODO could do some checking

        centroid.pose.position.x += obj.pose.pose.pose.position.x * obj.confidence;
        centroid.pose.position.y += obj.pose.pose.pose.position.y * obj.confidence;
        centroid.pose.position.z += obj.pose.pose.pose.position.z * obj.confidence;
        roll_acc                 += mtk::roll(obj.pose.pose.pose) * obj.confidence;
        pitch_acc                += mtk::pitch (obj.pose.pose.pose) * obj.confidence;
        yaw_acc                  += mtk::yaw(obj.pose.pose.pose) * obj.confidence;

        confidence_acc += obj.confidence;
      }

      centroid.pose.position.x /= confidence_acc;
      centroid.pose.position.y /= confidence_acc;
      centroid.pose.position.z /= confidence_acc;
      centroid.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_acc/confidence_acc,
                                                                          pitch_acc/confidence_acc,
                                                                          yaw_acc/confidence_acc);
      confidence = confidence_acc/(double)objects.size();
    }

    std::string getName() const
    {
      std::map<std::string, unsigned int> key_occurences;
      for (const object_recognition_msgs::RecognizedObject& obj: objects)
        //if (key_occurences.
        key_occurences[obj.type.key]++;

      std::string commonest_key;
      std::map<std::string, unsigned int>::iterator it;
      for(it = key_occurences.begin(); it != key_occurences.end(); it++)
      {
        if (it->second > key_occurences[commonest_key])
          commonest_key = it->first;
      }
      return commonest_key;
    }

    object_recognition_msgs::ObjectType getType() const
    {
      std::string commonest_key = getName();
      for (const object_recognition_msgs::RecognizedObject& obj: objects)
        if (obj.type.key == commonest_key)
          return obj.type;

      return object_recognition_msgs::ObjectType();
    }

  private:
    double confidence;
    geometry_msgs::PoseStamped centroid;
    std::vector<object_recognition_msgs::RecognizedObject> objects;
  };

};

};  // namespace turtlebot_arm_object_manipulation

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_detection_action_server");

  turtlebot_arm_object_manipulation::ObjectDetectionServer server("object_detection");
  ros::spin();

  return 0;
}
