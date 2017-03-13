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

#include <actionlib/client/simple_action_client.h>
#include <simple_arm_server/MoveArmAction.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <interactive_markers/interactive_marker_server.h>

#include <vector>


using namespace visualization_msgs;

const std::string arm_link = "/arm_base_link";
const double gripper_open = 0.04;
const double gripper_closed = 0.024;

const double z_up = 0.08;
const double z_down = -0.04;

const double block_size = 0.0127;

// Block storage
class Block
{
  public:
    int id;
    bool active;
    double x;
    double y;
    
    Block(const Block& b) : id(b.id), active(b.active), x(b.x), y(b.y) {}
    Block(int _id, double _x, double _y) : id(_id) , active(true), x(_x), y(_y) {}

    std::string getName()
    {
      std::stringstream conv;
      conv << id;
      return std::string("block") + conv.str(); 
    }
};


class BlockManipulation
{
private:
  interactive_markers::InteractiveMarkerServer server;
  actionlib::SimpleActionClient<simple_arm_server::MoveArmAction> client_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::TransformListener tf_listener_;
  int markers_;
  int moving_;
  int skip_;
  float x_, y_;
  std::string last_block_;
  std::vector<Block> marker_names_;
  
  ros::NodeHandle nh;

public:

  BlockManipulation() : server("block_controls"), client_("move_arm", true)
  {
    // create marker server
    markers_ = 0;
    last_block_ = std::string("");

    ros::Duration(0.1).sleep();
    
    skip_ = 0;

    // open gripper
    simple_arm_server::MoveArmGoal goal;
    simple_arm_server::ArmAction grip;
    grip.type = simple_arm_server::ArmAction::MOVE_GRIPPER;
    grip.command = gripper_open;
    goal.motions.push_back(grip);
    goal.header.frame_id = arm_link;
    client_.sendGoal(goal);
    client_.waitForResult(/*ros::Duration(30.0)*/);

    // subscribe to point cloud
    sub_ = nh.subscribe("/camera/depth_registered/points", 1, &BlockManipulation::cloudCb, this);
    pub_ = nh.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    server.applyChanges();
    
    ROS_INFO("Finished initializing.");
  }

  // Move the real block!
  void moveBlock( const InteractiveMarkerFeedbackConstPtr &feedback )
  {
    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM("Staging " << feedback->marker_name);     
        x_ = feedback->pose.position.x;
        y_ = feedback->pose.position.y;
        last_block_ = feedback->marker_name;
        break;
   
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        moving_ = true;
        ROS_INFO_STREAM("Now moving " << feedback->marker_name); 

        simple_arm_server::MoveArmGoal goal;
        simple_arm_server::ArmAction action;
        
        /* arm straight up */
        tf::Quaternion temp;
        temp.setRPY(0,1.57,0);
        action.goal.orientation.x = temp.getX();
        action.goal.orientation.y = temp.getY();
        action.goal.orientation.z = temp.getZ();
        action.goal.orientation.w = temp.getW();

        /* hover over */
        action.goal.position.x = x_;
        action.goal.position.y = y_;
        action.goal.position.z = z_up;
        goal.motions.push_back(action);
        action.move_time.sec = 1.5;

        /* go down */
        action.goal.position.z = z_down;
        goal.motions.push_back(action);
        action.move_time.sec = 1.5;

        /* close gripper */
        simple_arm_server::ArmAction grip;
        grip.type = simple_arm_server::ArmAction::MOVE_GRIPPER;
        grip.command = gripper_closed;
        grip.move_time.sec = 1.0;
        goal.motions.push_back(grip);

        /* go up */
        action.goal.position.z = z_up;
        goal.motions.push_back(action);
        action.move_time.sec = 0.25;

        /* hover over */
        action.goal.position.x = feedback->pose.position.x;
        action.goal.position.y = feedback->pose.position.y;
        action.goal.position.z = z_up;
        goal.motions.push_back(action);
        action.move_time.sec = 1.5;

        /* go down */
        action.goal.position.z = z_down;
        goal.motions.push_back(action);
        action.move_time.sec = 1.5;

        /* open gripper */
        grip.command = gripper_open;
        goal.motions.push_back(grip);

        /* go up */
        action.goal.position.z = z_up;
        goal.motions.push_back(action);
        action.move_time.sec = 0.25;
      
        goal.header.frame_id = arm_link;
        client_.sendGoal(goal);
        client_.waitForResult(/*ros::Duration(30.0)*/);
        /* update location */ 
        for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); it++)
        {
          if( it->getName() == feedback->marker_name )
          {
            it->x = feedback->pose.position.x;
            it->y = feedback->pose.position.y;
            break;
          }
        }

        moving_ = false;
        break;
    }
    
    server.applyChanges(); 
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
   
  // Add a new block
  void addBlock( float x, float y, float z, float rz, float r, float g, float b, int n)
  {
    InteractiveMarker marker;
    marker.header.frame_id = arm_link;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.scale = 0.03;
    
    Block block( n, x, y );
    marker_names_.push_back( block );
    marker.name = block.getName(); 
    marker.description = "Another block";

    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    marker.controls.push_back( control );
    
    control.markers.push_back( makeBox(marker, r, g, b) );
    control.always_visible = true;
    marker.controls.push_back( control );
    

    server.insert( marker );
    server.setCallback( marker.name, boost::bind( &BlockManipulation::moveBlock, this, _1 ));
  }

  // Process an incoming cloud
  void cloudCb ( const sensor_msgs::PointCloud2ConstPtr& msg )
  {
    if( (skip_++)%15 != 0 ) return;
    
    // convert to PCL
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg (*msg, cloud);
    
    // transform to base_link
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    tf_listener_.waitForTransform(std::string(arm_link), cloud.header.frame_id, cloud.header.stamp, ros::Duration(1.0));
    if (!pcl_ros::transformPointCloud (std::string(arm_link), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR ("Error converting to base_link");
      return;
    }

    // drop things on ground
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed); 
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_down - 0.01, z_down + block_size*2);
    pass.filter(*cloud_filtered);
    if( cloud_filtered->points.size() == 0 ){
      ROS_ERROR("0 points left");
      return;
    }else
      ROS_INFO("Filtered, %d points left", (int) cloud_filtered->points.size());
    pub_.publish(*cloud_filtered);

    // cluster
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);
    
    std::vector<pcl::PointIndices> clusters;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(20);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(clusters);

    // need to delete old blocks
    for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); it++)
    {
      it->active = false;
    }

    // for each cluster, see if it is a block
    for (size_t c = 0; c < clusters.size (); ++c)
    {  
      // find cluster centroid/color
      float x = 0; float y = 0; float z = 0; int r = 0; int g = 0; int b = 0;
      for (size_t i = 0; i < clusters[c].indices.size(); i++)
      {
          int j = clusters[c].indices[i];
          x += cloud_filtered->points[j].x;
          y += cloud_filtered->points[j].y;
          z += cloud_filtered->points[j].z;
          
          r += cloud_filtered->points[j].r;
          g += cloud_filtered->points[j].g;
          b += cloud_filtered->points[j].b;
      }
      x = x/clusters[c].indices.size();
      y = y/clusters[c].indices.size();
      z = z/clusters[c].indices.size();
      
      r = r/clusters[c].indices.size();
      g = g/clusters[c].indices.size();
      b = b/clusters[c].indices.size();

      bool new_ = true;
      // see if we have it detected
      for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); it++)
      {
        if( (fabs(it->x - x) < block_size*2) &&
            (fabs(it->y - y) < block_size*2) )
        {
          new_ = false;
          it->active = true;
          break;
        }
      }

      if (new_){
        // else, add new block
        addBlock( x, y, z - block_size, 0.0, (float) r/255.0, (float) g/255.0, (float) b/255.0, markers_++ );
      }
    }

    // need to delete old blocks
    for( std::vector<Block>::iterator it=marker_names_.begin(); it < marker_names_.end(); )
    {
      if(it->active or it->getName() == last_block_){
        it++;
      }else{
        server.erase( it->getName() );
        it = marker_names_.erase(it);
      }
    }

    server.applyChanges();
}

};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "block_manipulation");

  BlockManipulation manip;

  // everything is done in cloud callback, just spin
  ros::spin();
}

