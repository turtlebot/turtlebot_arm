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

#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// auxiliary libraries
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

  // Get object information from database service and keep in a map
  ros::ServiceClient obj_info_srv_;
  std::map<std::string, object_recognition_msgs::ObjectInformation> objs_info_;

  // Publishers and subscribers
  ros::Subscriber table_sub_;

  ros::Publisher clear_objs_pub_;
  ros::Publisher clear_table_pub_;

  typedef struct
  {
    double size_x;
    double size_y;
    double center_x;
    double center_y;
    double yaw;
  } TableDescriptor;

  std::vector<geometry_msgs::Pose> table_poses_;
  std::vector<TableDescriptor>     table_params_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf::TransformListener tf_listener_;

  // Parameters from goal
  std::string arm_link_;
  
  // Object detection and classification constants
  const double   CONFIDENCE_THRESHOLD  = 0.85;   // minimum confidence required to accept an object
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
    if (! ork_ac_.waitForServer(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] tabletop/recognize_objects action server not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }

    ROS_INFO("[object detection] tabletop/recognize_objects action server started; ready for sending goals.");

    // Wait for the get object information service (mandatory, as we need to know objects' mesh)
    obj_info_srv_ = nh_.serviceClient<object_recognition_msgs::GetObjectInformation>("get_object_info");
    if (! obj_info_srv_.waitForExistence(ros::Duration(60.0)))
    {
      ROS_ERROR("[object detection] Get object information service not available after 1 minute");
      ROS_ERROR("[object detection] Shutting down node...");
      throw;
    }

    // Register the goal and feedback callbacks.
    od_as_.registerGoalCallback(boost::bind(&ObjectDetectionServer::goalCB, this));
    od_as_.registerPreemptCallback(boost::bind(&ObjectDetectionServer::preemptCB, this));
    
    od_as_.start();
    
    // Subscribe to detected tables array
    table_sub_ = nh_.subscribe("tabletop/table_array", 1, &ObjectDetectionServer::tableCb, this);

    // Publish empty objects and table to clear ORK RViz visualizations
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

    // Clear results from previous goals
    table_poses_.clear();
    table_params_.clear();
    result_.obj_names.clear();

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
      ROS_INFO("[object detection] Succeeded! %d objects detected", added_objects);

      // Add also the table as a collision object, so it gets filtered out from MoveIt! octomap
      if (table_poses_.size() > 0)
        addTable(table_poses_, table_params_);
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
      ROS_INFO("[object detection] Succeeded, but couldn't find any object");
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
    // We only take the first table in the message, assuming it is always the best match and so the same
    // table... very risky, to say the least. TODO: assure we always accumulate data over the same table
    object_recognition_msgs::Table table = msg.tables[0];
    geometry_msgs::Pose table_pose = table.pose;
    // Tables often have orientations with all-nan values, but assertQuaternionValid lets them go!
    if (std::isnan(table.pose.orientation.x) ||
        std::isnan(table.pose.orientation.y) ||
        std::isnan(table.pose.orientation.z) ||
        std::isnan(table.pose.orientation.w))
    {
      ROS_WARN("[object detection] Table discarded as its orientation has nan values");
      return;
    }

    if (! transformPose(msg.header.frame_id, arm_link_, table.pose, table_pose))
    {
      return;
    }

    if ((std::abs(mtk::roll(table_pose)) < M_PI/10.0) && (std::abs(mtk::pitch(table_pose)) < M_PI/10.0))
    {
      // Only consider tables within +/-18 degrees away from the horizontal plane
      table_poses_.push_back(table_pose);
      table_params_.push_back(getTableParams(table.convex_hull));
    }
    else
    {
      ROS_WARN("[object detection] Table with pose [%s] discarded: %.2f radians away from the horizontal",
               mtk::pose2str3D(table_pose).c_str(),
               std::max(std::abs(mtk::roll(table_pose)), std::abs(mtk::pitch(table_pose))));
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

      try
      {
        // Compose object name with the name provided by the database plus an index, starting with [1]
        object_recognition_msgs::ObjectInformation obj_info = getObjInfo(bin.getType());
        obj_name_occurences[obj_info.name]++;
        std::stringstream sstream;
        sstream << obj_info.name << " [" << obj_name_occurences[obj_info.name] << "]";
        std::string obj_name = sstream.str();

        ROS_DEBUG("Bin %d with centroid [%s] and %d objects added as object '%s'",
                   bin.id, mtk::pose2str3D(bin.getCentroid()).c_str(), bin.countObjects(), obj_name.c_str());

        geometry_msgs::Pose out_pose;
        transformPose(bin.getCentroid().header.frame_id, arm_link_, bin.getCentroid().pose, out_pose);
        ROS_INFO("[object detection] Adding '%s' object at %s",
                 obj_name.c_str(), mtk::point2str3D(out_pose.position).c_str());

        result_.obj_names.push_back(obj_name);

        moveit_msgs::CollisionObject co;
        co.id = obj_name;
        co.header.frame_id = arm_link_;
        co.operation = moveit_msgs::CollisionObject::ADD;

        if (obj_name.find("cube 2.5") != std::string::npos)
        {
          // XXX, TODO :  temporal hack because bloody meshes don't clean the octomap!  (anyway, as I support both mechanism, it's also useful for debug)
          // Problem discussed in this moveit-users group thread: https://groups.google.com/forum/?fromgroups#!topic/moveit-users/-GuBSnvCYbM
          co.primitives.resize(1);
          co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
          co.primitives[0].dimensions.resize(3);
          co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.025;
          co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.025;
          co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.025;
          out_pose.position.z += 0.0125;
          co.primitive_poses.push_back(out_pose);
        }
        else
        {
          co.meshes.resize(1, obj_info.ground_truth_mesh);
          co.mesh_poses.push_back(out_pose);
        }

        collision_objects.push_back(co);

        // Provide a random color to the collision object
        moveit_msgs::ObjectColor oc;
        oc.id = co.id;
        oc.color = getRandColor(1.0);
        ps.object_colors.push_back(oc);
      }
      catch (...)
      {
        continue;
      }
    }

    if (collision_objects.size() > 0)
    {
      planning_scene_interface_.addCollisionObjects(collision_objects, ps.object_colors);
    }

    return collision_objects.size();
  }

  void addTable(const std::vector<geometry_msgs::Pose>& table_poses,
                const std::vector<TableDescriptor>& table_params)
  {
    // Add the table as a collision object into the world, so it gets excluded from the collision map

    // We calculate table size, centroid and orientation as the median of the accumulated convex hulls
    std::vector<double> table_center_x;
    std::vector<double> table_center_y;
    std::vector<double> table_size_x;
    std::vector<double> table_size_y;
    std::vector<double> table_yaw;

    for (const TableDescriptor& table: table_params)
    {
      table_center_x.push_back(table.center_x);
      table_center_y.push_back(table.center_y);
      table_size_x.push_back(table.size_x);
      table_size_y.push_back(table.size_y);
      table_yaw.push_back(table.yaw);
    }

    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = arm_link_;
    co.id = "table";

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(3);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = median(table_size_x);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = median(table_size_y);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.01;  // arbitrarily set to 1 cm
    co.primitive_poses.resize(1);

    // Calculate table's pose as the centroid of all accumulated poses;
    // as yaw we use the one estimated for the convex hull
    double roll_acc = 0.0, pitch_acc = 0.0;//, yaw_acc = 0.0;
    for (const geometry_msgs::Pose& pose: table_poses)
    {
      co.primitive_poses[0].position.x += pose.position.x;
      co.primitive_poses[0].position.y += pose.position.y;
      co.primitive_poses[0].position.z += pose.position.z;
      roll_acc                         += mtk::roll(pose);
      pitch_acc                        += mtk::pitch (pose);
      // yaw_acc                          += mtk::yaw(pose);
    }

    co.primitive_poses[0].position.x = co.primitive_poses[0].position.x / (double)table_poses.size();
    co.primitive_poses[0].position.y = co.primitive_poses[0].position.y / (double)table_poses.size();
    co.primitive_poses[0].position.z = co.primitive_poses[0].position.z / (double)table_poses.size();
    co.primitive_poses[0].orientation = tf::createQuaternionMsgFromRollPitchYaw(roll_acc/(double)table_poses.size(),
                                                                                pitch_acc/(double)table_poses.size(),
                                                                                median(table_yaw));
    // Displace the table center according to the centroid of its convex hull
    co.primitive_poses[0].position.x += median(table_center_x);
    co.primitive_poses[0].position.y += median(table_center_y);
    co.primitive_poses[0].position.z -= co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]/2.0;

    ROS_INFO("[object detection] Adding a table at %s as a collision object, based on %u observations",
             mtk::point2str3D(co.primitive_poses[0].position).c_str(), table_poses.size());
    planning_scene_interface_.addCollisionObjects(std::vector<moveit_msgs::CollisionObject>(1, co));
  }

  TableDescriptor getTableParams(std::vector<geometry_msgs::Point> convex_hull)
  {
    // Calculate centroid, size and orientation for the given 2D convex hull
    // Algorithm adapted from here: http://www.geometrictools.com/Documentation/MinimumAreaRectangle.pdf
    TableDescriptor table;
    double min_area = std::numeric_limits<double>::max();

    for (size_t i0 = convex_hull.size() - 1, i1 = 0; i1 < convex_hull.size(); i0 = i1++)
    {
      convex_hull[i0].z = convex_hull[i1].z = 0; // z-coordinate is ignored

      tf::Point origin, U0, U1, D;
      tf::pointMsgToTF(convex_hull[i0], origin);
      tf::pointMsgToTF(convex_hull[i1], U0); U0 -= origin;
      U0.normalize(); // length of U0 is 1
      U1 = tf::Point(-U0.y(), U0.x(), 0.0); // - perpendicular vector of U0
      U1.normalize(); // length of U1 is 1
      double min0 = 0, max0 = 0; // projection onto U0 − axis is [min0, max0]
      double min1 = 0, max1 = 0; // projection onto U1 − axis is [min1, max1], min1 = 0 is guaranteed  TODO si?????
      for (size_t j = 0; j < convex_hull.size(); ++j)
      {
        convex_hull[j].z = 0; // z-coordinate is ignored

        tf::pointMsgToTF(convex_hull[j], D); D -= origin;
        double dot = U0.dot(D);
        if (dot < min0)
          min0 = dot;
        else if (dot > max0)
          max0 = dot;
        dot = U1.dot(D);
        if (dot < min1)
          min1 = dot;
        else if (dot > max1)
          max1 = dot;
      }
      double area = (max0 - min0) * (max1 - min1);

      if (area < min_area)
      {
        tf::Point center(origin + ((min0 + max0)/2.0) * U0 + ((min1 + max1)/2.0) * U1);
        table.center_x =   center.y();
        table.center_y = - center.x();
        table.size_x = max0 - min0;
        table.size_y = max1 - min1;
        table.yaw = std::atan2(U1.y(), U1.x());
        if (table.yaw > 0.0)
          table.yaw -= M_PI;
        min_area = area;
      }
    }
    ROS_DEBUG("Table parameters: pose [%f, %f, %f   %f], size [%f, %f]", table.center_x, table.center_y, table.yaw, table.size_x, table.size_y);

    return table;
  }

  double median(std::vector<double>& values)
  {
    std::vector<double>::iterator first = values.begin();
    std::vector<double>::iterator last = values.end();
    std::vector<double>::iterator middle = first + (last - first) / 2;
    std::nth_element(first, middle, last); // short values till middle one
    return *middle;
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

  const object_recognition_msgs::ObjectInformation& getObjInfo(const object_recognition_msgs::ObjectType& obj_type)
  {
    if (objs_info_.find(obj_type.key) == objs_info_.end() )
    {
      // Get object information from db using object's type
      object_recognition_msgs::GetObjectInformation srv;
      srv.request.type = obj_type;

      if (obj_info_srv_.call(srv))
      {
        ROS_DEBUG("Database information retrieved for object '%s'", obj_type.key.c_str());
        objs_info_[obj_type.key] = srv.response.information;
      }
      else
      {
        ROS_ERROR("Call to object information service with key '%s' failed", obj_type.key.c_str());
        throw;
      }
    }

    return objs_info_[obj_type.key];
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
