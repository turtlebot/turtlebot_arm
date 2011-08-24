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
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <turtlebot_block_manipulation/BlockDetectionAction.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>

/*const std::string arm_link = "/arm_base_link";
const double gripper_open = 0.04;
const double gripper_closed = 0.024;

const double z_up = 0.08;
const double z_down = -0.04;

const double block_size = 0.0127; */

const std::string block_topic = "/turtlebot_blocks";

class BlockDetectionServer
{
private:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<turtlebot_block_manipulation::BlockDetectionAction> as_;
  std::string action_name_;
  turtlebot_block_manipulation::BlockDetectionFeedback feedback_;
  turtlebot_block_manipulation::BlockDetectionResult result_;
  turtlebot_block_manipulation::BlockDetectionGoalConstPtr goal_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  
  tf::TransformListener tf_listener_;
  
  // Parameters
  std::string arm_link;
  double block_size;
  double table_height;
  
  ros::Publisher block_pub_;
  
public:
  BlockDetectionServer(const std::string name) : 
    nh_(), as_(nh_, name, false), action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&BlockDetectionServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&BlockDetectionServer::preemptCB, this));
    
    as_.start();
    
    // subscribe to point cloud
    sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &BlockDetectionServer::cloudCb, this);
    pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);
    
    block_pub_ = nh_.advertise< geometry_msgs::PoseArray >(block_topic, 1, true);
  }

  void goalCB()
  {
    ROS_INFO("[block detection] Received goal!");
    // accept the new goal
    result_.blocks.poses.clear();
    
    goal_ = as_.acceptNewGoal();
    
    block_size = goal_->block_size;
    table_height = goal_->table_height;
    arm_link = goal_->frame;
    
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void cloudCb ( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    if (!as_.isActive()) return;
    
    // convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg (*msg, cloud);
    
    // transform to base_link
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    tf_listener_.waitForTransform(std::string(arm_link), cloud.header.frame_id, cloud.header.stamp, ros::Duration(1.0));
    if (!pcl_ros::transformPointCloud (std::string(arm_link), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR ("Error converting to desired frame");
      return;
    }

    // drop things on ground
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed); 
    pass.setFilterFieldName("z");
    pass.setFilterLimits(table_height - 0.01, table_height + block_size*2);
    pass.filter(*cloud_filtered);
    if( cloud_filtered->points.size() == 0 ){
      ROS_ERROR("0 points left");
      return;
    }else
      ROS_INFO("Filtered, %d points left", (int) cloud_filtered->points.size());
    pub_.publish(*cloud_filtered);

    // cluster
    pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);
    
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(clusters);

    // for each cluster, see if it is a block
    for (size_t c = 0; c < clusters.size (); ++c)
    {  
      // find cluster centroid/color
      float x = 0; float y = 0; float z = 0;
      for (size_t i = 0; i < clusters[c].indices.size(); i++)
      {
          int j = clusters[c].indices[i];
          x += cloud_filtered->points[j].x;
          y += cloud_filtered->points[j].y;
          z += cloud_filtered->points[j].z;
      }
      x = x/clusters[c].indices.size();
      y = y/clusters[c].indices.size();
      z = z/clusters[c].indices.size();
      
      // else, add new block
      addBlock( x, y, z - block_size);
    }
    
    if (result_.blocks.poses.size() > 0)
    {
      as_.setSucceeded(result_);
      block_pub_.publish(result_.blocks);
    }
    //else
    //  as_.setAborted(result_);
  }
    
  void addBlock(float x, float y, float z)
  {
    geometry_msgs::Pose block_pose;
    block_pose.position.x = x;
    block_pose.position.y = y;
    block_pose.position.z = z;
    
    block_pose.orientation.x = 0;
    block_pose.orientation.y = 0;
    block_pose.orientation.z = 0;
    block_pose.orientation.w = 1;
    
    result_.blocks.poses.push_back(block_pose);
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "block_detection_action_server");

  BlockDetectionServer server("block_detection");
  ros::spin();

  return 0;
}

