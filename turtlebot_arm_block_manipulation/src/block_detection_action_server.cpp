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

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_arm_block_manipulation/BlockDetectionAction.h>

#include <tf/transform_listener.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <cmath>
#include <algorithm>

namespace turtlebot_arm_block_manipulation
{

class BlockDetectionServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_arm_block_manipulation::BlockDetectionAction> as_;
  std::string action_name_;
  turtlebot_arm_block_manipulation::BlockDetectionFeedback feedback_;
  turtlebot_arm_block_manipulation::BlockDetectionResult result_;
  turtlebot_arm_block_manipulation::BlockDetectionGoalConstPtr goal_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  // We use the planning_scene_interface::PlanningSceneInterface to manipulate the world
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  tf::TransformListener tf_listener_;
  
  // Parameters from goal
  std::string arm_link_;
  double block_size_;
  double table_height_;
  
  ros::Publisher block_pub_;
  ros::Publisher c_obj_pub_;
  
  // Parameters from node
  std::vector<double> table_pose_;

  // Constants
  const double TABLE_SIZE_X = 0.5;
  const double TABLE_SIZE_Y = 1.0;
  const double TABLE_SIZE_Z = 0.05;

  
public:
  BlockDetectionServer(const std::string name) : 
    nh_("~"), as_(name, false), action_name_(name)
  {
    // Load parameters from the server.
    if ((nh_.getParam("table_pose", table_pose_) == true) && (table_pose_.size() != 3))
    {
      ROS_ERROR("Invalid table_pose vector size; must contain 3 values (x, y, z); ignoring");
      table_pose_.clear();
    }
    
    // Register the goal and feeback callbacks.
    as_.registerGoalCallback(boost::bind(&BlockDetectionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&BlockDetectionServer::preemptCB, this));
    
    as_.start();
    
    // Subscribe to point cloud
    sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &BlockDetectionServer::cloudCb, this);

    // Publish the filtered point cloud for debug purposes
    pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Public detected blocks poses
    block_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/turtlebot_blocks", 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[block detection] Received goal!");
    // accept the new goal
    result_.blocks.poses.clear();
    
    goal_ = as_.acceptNewGoal();
    
    block_size_ = goal_->block_size;
    table_height_ = goal_->table_height;
    arm_link_ = goal_->frame;

    result_.blocks.header.frame_id = arm_link_;

    // Add the table as a collision object, so it gets filtered out from MoveIt! octomap
    // I let it optional, as I don't know if this will work in most cases, honesty speaking
    if (table_pose_.size() == 3)
    {
      if (std::abs(table_height_ - table_pose_[2]) > 0.05)
        ROS_WARN("The table height (%f) passed with goal and table_pose[2] parameter (%f) " \
                 "should be very similar", table_height_, table_pose_[2]);
      addTable();
    }
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Only do this if we're actually actively working on a goal.
    if (!as_.isActive())
      return;

    result_.blocks.header.stamp = msg->header.stamp;

    // convert to PCL
    pcl::PointCloud < pcl::PointXYZRGB > cloud;
    pcl::fromROSMsg(*msg, cloud);

    // transform to whatever frame we're working in, probably the arm frame.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);

    tf_listener_.waitForTransform(std::string(arm_link_), cloud.header.frame_id,
                                  ros::Time(cloud.header.stamp), ros::Duration(1.0));
    if (!pcl_ros::transformPointCloud(std::string(arm_link_), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR("Error converting to desired frame");
      return;
    }

    // Create the segmentation object for the planar model and set all the parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.005);

    // Limit to things we think are roughly at the table height.
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("z");

    pass.setFilterLimits(table_height_ - 0.05, table_height_ + block_size_ + 0.05);
    pass.filter(*cloud_filtered);
    if (cloud_filtered->points.size() == 0)
    {
      ROS_ERROR("0 points left");
      return;
    }
    else
      ROS_INFO("Filtered, %d points left", (int ) cloud_filtered->points.size());

    int nr_points = cloud_filtered->points.size ();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
      }

      std::cout << "Inliers: " << (inliers->indices.size()) << std::endl;

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Write the planar inliers to disk
      extract.filter(*cloud_plane);
      std::cout << "PointCloud representing the planar component: "
                << cloud_plane->points.size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered);
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.005);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pub_.publish(cloud_filtered);

    // for each cluster, see if it is a block
    for (size_t c = 0; c < cluster_indices.size (); ++c)
    {  
      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0;
      float ymin = 0; float ymax = 0;
      float zmin = 0; float zmax = 0;

      for (size_t i = 0; i < cluster_indices[c].indices.size(); i++)
      {
          int j = cluster_indices[c].indices[i];
          float x = cloud_filtered->points[j].x;
          float y = cloud_filtered->points[j].y;
          float z = cloud_filtered->points[j].z;
          if (i == 0)
          {
            xmin = xmax = x;
            ymin = ymax = y;
            zmin = zmax = z;
          }
          else
          {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            ymin = std::min(ymin, y);
            ymax = std::max(ymax, y);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
          }    
      }
      
      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;
      float zside = zmax-zmin;
      
      const float tol = 0.01; // 1 cm error tolerance
      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      // z must be equal to or smaller than blocksize
      if (xside > block_size_ - tol && xside < block_size_*sqrt(2) + tol &&
          yside > block_size_ - tol && yside < block_size_*sqrt(2) + tol &&
          zside > tol && zside < block_size_ + tol)
      {
        float x = xmin + xside/2.0;
        float y = ymin + yside/2.0;
        float z = zmax - block_size_/2.0;

        // Discard blocks outside the table limits (normally something detected on the robot)
        // Note that table pose x marks the table border closest to arm_link_ frame, not its center
        if ((table_pose_.size() == 3) &&
            ((x < table_pose_[0]) || (x > table_pose_[0] + TABLE_SIZE_X) ||
             (y < table_pose_[1] - TABLE_SIZE_Y/2.0) || (y > table_pose_[1] + TABLE_SIZE_Y/2.0)))
        {
          ROS_DEBUG_STREAM("Block at [" << x << ", " << y << "] outside table limits ["
                         << table_pose_[0] - TABLE_SIZE_X/2.0 << ", " << table_pose_[0] + TABLE_SIZE_X/2.0 << ", "
                         << table_pose_[1] - TABLE_SIZE_Y/2.0 << ", " << table_pose_[1] + TABLE_SIZE_Y/2.0);
          continue;
        }

        // If so, then figure out the position and the orientation of the block
        float angle = atan(block_size_/((xside + yside)/2));
        
        if (yside < block_size_)
          angle = 0.0;
        
        ROS_INFO_STREAM("xside: " << xside << " yside: " << yside << " zside " << zside << " angle: " << angle);
        // Then add it to our set
        ROS_INFO_STREAM("Adding a new block at [" << x << ", " << y << ", " << z << ", " << angle << "]");
        addBlock(x, y, z, angle);
      }
    }
    
    if (result_.blocks.poses.size() > 0)
    {
      as_.setSucceeded(result_);
      block_pub_.publish(result_.blocks);
      ROS_INFO("[block detection] Set as succeeded!");
    }
    else
      ROS_INFO("[block detection] Couldn't find any blocks this iteration!");
    //  as_.setAborted(result_);
  }


private:

  void addBlock(float x, float y, float z, float angle)
  {
    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;
    
    Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));
    
    block_pose.orientation.x = quat.x();
    block_pose.orientation.y = quat.y();
    block_pose.orientation.z = quat.z();
    block_pose.orientation.w = quat.w();
    
    result_.blocks.poses.push_back(block_pose);
  }

  void addTable()
  {
    // Add the table as a collision object into the world, so it gets excluded from the collision map
    // As the blocks are small, they should also be excluded (assuming that padding_offset parameter on
    // octomap sensor configuration is equal or bigger than block size)
    moveit_msgs::CollisionObject co;
    co.header.stamp = ros::Time::now();
    co.header.frame_id = arm_link_;

    co.id = "table";
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>(1, co.id));

    co.operation = moveit_msgs::CollisionObject::ADD;
    co.primitives.resize(1);
    co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co.primitives[0].dimensions.resize(3);
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = TABLE_SIZE_X;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = TABLE_SIZE_Y;
    co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = TABLE_SIZE_Z;
    co.primitive_poses.resize(1);
    co.primitive_poses[0].position.x = table_pose_[0] + TABLE_SIZE_X/2.0;
    co.primitive_poses[0].position.y = table_pose_[1];
    co.primitive_poses[0].position.z = table_pose_[2] - TABLE_SIZE_Z/2.0;

    ROS_INFO("Add the table as a collision object into the world");
    std::vector<moveit_msgs::CollisionObject> collision_objects(1, co);
    planning_scene_interface_.addCollisionObjects(collision_objects);
  }

};

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_detection_action_server");

  turtlebot_arm_block_manipulation::BlockDetectionServer server("block_detection");
  ros::spin();

  return 0;
}
