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
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/registration.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <turtlebot_kinect_arm_calibration/detect_calibration_pattern.h>

using namespace std;
using namespace Eigen;

/*const std::string fixed_frame = "/base_link";
const std::string camera_frame = "/camera_link";
//const std::string base_frame = "/target_frame";
//const std::string camera_frame = "/kinect_rgb_optical_frame";
const std::string target_frame = "/calibration_pattern";
const std::string tip_frame = "/gripper_link";
const std::string touch_frame = "/gripper_touch";

const std::string camera_topic = "/camera/rgb/";
const std::string image_topic = camera_topic + "image_mono";
const std::string cloud_topic = camera_topic + "points";
const std::string info_topic = camera_topic + "camera_info";

//const std::string image_topic = "/image_throttled";
//const std::string cloud_topic = "/cloud_throttled";

const int checkerboard_width = 6;
const int checkerboard_height = 7;
const double checkerboard_grid = 0.027; */


// TODO: replace
tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
    btMatrix3x3 btm;
    btm.setValue(trans(0,0),trans(0,1),trans(0,2),
                 trans(1,0),trans(1,1),trans(1,2),
                 trans(2,0),trans(2,1),trans(2,2));
    btTransform ret;
    ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
    ret.setBasis(btm);
    return ret;
}

Eigen::Matrix4f EigenFromTF(tf::Transform trans)
{
  Eigen::Matrix4f out;
  btQuaternion quat = trans.getRotation();
  btVector3 origin = trans.getOrigin();
  
  Eigen::Quaternionf quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3f origin_out(origin.x(), origin.y(), origin.z());
  
  out.topLeftCorner<3,3>() = quat_out.toRotationMatrix();
  out.topRightCorner<3,1>() = origin_out;
  out(3,3) = 1;
  
  return out;
}


class CalibrateKinectCheckerboard
{
    // Nodes and publishers/subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher detector_pub_;
    ros::Publisher physical_pub_;
  
    // Image and camera info subscribers;
    ros::Subscriber image_sub_; 
    ros::Subscriber info_sub_;

    // Structures for interacting with ROS messages
    cv_bridge::CvImagePtr bridge_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    image_geometry::PinholeCameraModel cam_model_;
    
    // Calibration objects
    PatternDetector pattern_detector_;
    
    // The optimized transform
    Eigen::Transform<float, 3, Eigen::Affine> transform_;
    
    // Gripper transform
    tf::Transform touch_transform;
    
    // Parameters for image callback
    // (Look into encapsulating these...)
    //PointVector true_points_;
    
    // Visualization for markers
    pcl::PointCloud<pcl::PointXYZ> detector_points_;
    pcl::PointCloud<pcl::PointXYZ> ideal_points_;
    pcl::PointCloud<pcl::PointXYZ> image_points_;
    pcl::PointCloud<pcl::PointXYZ> physical_points_;
    
    // Have we calibrated the camera yet?
    bool calibrated;
    
    ros::Timer timer_;
    
    // Parameters
    std::string fixed_frame;
    std::string camera_frame;
    std::string target_frame;
    std::string tip_frame;
    std::string touch_frame;
    
    std::string camera_topic;
    std::string image_topic;
    std::string cloud_topic;
    std::string info_topic;
    
    int checkerboard_width;
    int checkerboard_height;
    double checkerboard_grid;

public:
  CalibrateKinectCheckerboard()
    : nh_("~"), it_(nh_), calibrated(false)
  {
    // Load parameters from the server.
    nh_.param<std::string>("fixed_frame", fixed_frame, "/base_link");
    nh_.param<std::string>("camera_frame", camera_frame, "/camera_link");
    nh_.param<std::string>("target_frame", target_frame, "/calibration_pattern");
    nh_.param<std::string>("tip_frame", tip_frame, "/gripper_link");
    nh_.param<std::string>("touch_frame", touch_frame, "/gripper_touch");
    nh_.param<std::string>("camera_topic", camera_topic, "/camera/rgb/");
    
    nh_.param<int>("checkerboard_width", checkerboard_width, 6);
    nh_.param<int>("checkerboard_height", checkerboard_height, 7);
    nh_.param<double>("checkerboard_grid", checkerboard_grid, 0.027);
    
    image_topic = camera_topic + "image_mono";
    cloud_topic = camera_topic + "points";
    info_topic = camera_topic + "camera_info";

    // Set pattern detector sizes
    pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    
    transform_.translation().setZero();
    transform_.matrix().topLeftCorner<3, 3>() = Quaternionf().setIdentity().toRotationMatrix();
    
    pub_ = it_.advertise("calibration_pattern_out", 1);
    
    // Create subscriptions
    info_sub_ = nh_.subscribe(info_topic, 1, &CalibrateKinectCheckerboard::infoCallback, this);
    touch_transform.setOrigin( tf::Vector3(0, -0.016, -0.019) );
    touch_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    
    // Also publishers
    detector_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("detector_cloud", 1);
    physical_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("physical_points_cloud", 1);
    
    // Create ideal points
    ideal_points_.push_back( pcl::PointXYZ(0, 0, 0) );
    ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, 0, 0) );
    ideal_points_.push_back( pcl::PointXYZ(0, (checkerboard_height-1)*checkerboard_grid, 0) );
    ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, (checkerboard_height-1)*checkerboard_grid, 0) );
  }

  void publishTFCallback(const ros::TimerEvent&)
  {
    tf_broadcaster_.sendTransform(tf::StampedTransform(touch_transform, ros::Time::now(), tip_frame, touch_frame));
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    if (calibrated)
      return;
    cam_model_.fromCameraInfo(info_msg);
    pattern_detector_.setCameraMatrices(cam_model_.intrinsicMatrix(), cam_model_.distortionCoeffs());
    
    calibrated = true;
    image_sub_ = nh_.subscribe(image_topic, 1, &CalibrateKinectCheckerboard::imageCallback, this);
    
    cout << "Got image info!" << endl;
  }
  
  void pointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg)
  {
    sensor_msgs::ImagePtr image_msg(new sensor_msgs::Image);
    pcl::toROSMsg (*msg, *image_msg);
  
    imageCallback(image_msg);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& image_msg)
  {
    // DEBUG
      
      
     /* physical_points_.header.frame_id = fixed_frame;
      physical_points_.resize(0);
      addPhysicalPoint();
      physical_pub_.publish(physical_points_); */
  
    try
    {
      bridge_ = cv_bridge::toCvCopy(image_msg, "mono8");
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("[calibrate] Failed to convert image");
      return;
    }
  
    Eigen::Vector3f translation;
    Eigen::Quaternionf orientation;
    
    if (!pattern_detector_.detectPattern(bridge_->image, translation, orientation))
    {
      ROS_INFO("[calibrate] Couldn't detect checkerboard, make sure it's visible in the image.");
      return;
    }
      
    // DEBUG
    pub_.publish(bridge_->toImageMsg());
    
    tf::Transform target_transform;
    tf::StampedTransform base_transform;
    try
    {
      ros::Time acquisition_time = image_msg->header.stamp;
      ros::Duration timeout(1.0 / 30.0);
      
      // Get base_link transform
      /*tf_listener_.waitForTransform(world_frame, base_frame,
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform(world_frame, base_frame,
                                   acquisition_time, base_transform); */
                                   
      target_transform.setOrigin( tf::Vector3(translation.x(), translation.y(), translation.z()) );
      target_transform.setRotation( tf::Quaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w()) );
      tf_broadcaster_.sendTransform(tf::StampedTransform(target_transform, image_msg->header.stamp, image_msg->header.frame_id, target_frame));
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    
    //convertIdealPointstoPointcloud();
    //publishCloud(detector_points_, target_transform, image_msg->header.frame_id);
    
    publishCloud(ideal_points_, target_transform, image_msg->header.frame_id);
    
    // Alright, now we have a btTransform...
    // Convert it to something we can use.
    // ObjectPose? Eigen::Transform?
    /*
    Eigen::Vector3f base_translation(base_transform.getOrigin().x(), 
                  base_transform.getOrigin().y(), base_transform.getOrigin().z());
    Eigen::Quaternionf base_orientation(base_transform.getRotation().w(), 
                  base_transform.getRotation().x(), base_transform.getRotation().y(), 
                  base_transform.getRotation().z()); */
  
    pcl_ros::transformPointCloud(ideal_points_, image_points_, target_transform);
    
    
    cout << "Got an image callback!" << endl;
    
    calibrate(image_msg->header.frame_id);
    
    ros::shutdown();
  }
  

  void publishCloud(pcl::PointCloud<pcl::PointXYZ> detector_points, tf::Transform &transform, const std::string frame_id)
  {
    // Display to rviz
    pcl::PointCloud<pcl::PointXYZ> transformed_detector_points;
    
    pcl_ros::transformPointCloud(detector_points, transformed_detector_points, transform);
    
    transformed_detector_points.header.frame_id = frame_id;
    detector_pub_.publish(transformed_detector_points);
  }
  
  bool calibrate(const std::string frame_id)
  {
    physical_points_.empty();
    physical_points_.header.frame_id = fixed_frame;
    cout << "Is the checkerboard correct? " << endl;
    cout << "Move edge of gripper to point 1 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 2 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 3 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    cout << "Move edge of gripper to point 4 in image and press Enter. " << endl;
    cin.ignore();
    addPhysicalPoint();
    
    Eigen::Matrix4f t;
    
    physical_pub_.publish(physical_points_);
    
    pcl::estimateRigidTransformationSVD( physical_points_, image_points_, t );
    //cout << "Does this transform seem right? " << endl;
    //cout << "Run this in the command line: " << endl;

    // TEMP: output

    // output cloud & board frame       
    tf::Transform transform = tfFromEigen(t), trans_full, camera_transform_unstamped;
    tf::StampedTransform camera_transform;
  
    cout << "Resulting transform (camera frame -> fixed frame): " << endl << t << endl << endl;
    //cout << "Quick test: this should be the same as above. " << endl << EigenFromTF(transform) << endl << endl;
    //printStaticTransform(t, frame_id, fixed_frame);
    
    
    try
    {
      tf_listener_.lookupTransform(frame_id, camera_frame, ros::Time(0), camera_transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return false;
    }

    camera_transform_unstamped = camera_transform;
    trans_full = camera_transform_unstamped.inverse()*transform;
    
    Eigen::Matrix4f t_full = EigenFromTF(trans_full);
    Eigen::Matrix4f t_full_inv = (Eigen::Transform<float,3,Affine>(t_full).inverse()).matrix();
    
    cout << "Resulting transform (fixed frame -> camera frame): " << endl << t_full << endl << endl;
    printStaticTransform(t_full_inv, fixed_frame, camera_frame);

    return true;
  }
  
  void printStaticTransform(Eigen::Matrix4f& transform, const std::string frame1, const std::string frame2)
  {
    Eigen::Quaternionf quat(transform.topLeftCorner<3,3>() );
    Eigen::Vector3f translation(transform.topRightCorner<3,1>() );
    
    cout << "rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms" << endl;
    cout << "rosrun tf static_transform_publisher " << translation.x() << " "
         << translation.y() << " " << translation.z() << " " 
         << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << " "
         << frame1 << " " << frame2 << " 100" << endl;
  }
  
  void addPhysicalPoint()
  {
    geometry_msgs::PointStamped pt, pt_out; // Initialized to (0,0,0)
    
    pt.point.x = -0.002;
    pt.point.y = -0.020; // probably subtract another 4 mm for width of gripper
    pt.point.z = -0.0185; // Is this even remotely right?
    pt.header.frame_id = tip_frame;
    try
    {
      tf_listener_.transformPoint(fixed_frame, pt, pt_out);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[calibrate] TF exception:\n%s", ex.what());
      return;
    }
    
    physical_points_.push_back(pcl::PointXYZ(pt_out.point.x, pt_out.point.y, pt_out.point.z));
  }

  void convertIdealPointstoPointcloud()
  {
    detector_points_.points.resize(pattern_detector_.ideal_points.size());
    for (unsigned int i=0; i < pattern_detector_.ideal_points.size(); i++)
    {
      cv::Point3f pt = pattern_detector_.ideal_points[i];
      detector_points_[i].x = pt.x; detector_points_[i].y = pt.y; detector_points_[i].z = pt.z; 
    }
  }
  
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "calibrate_kinect_base");
  
  CalibrateKinectCheckerboard cal;
  ros::spin();
}

