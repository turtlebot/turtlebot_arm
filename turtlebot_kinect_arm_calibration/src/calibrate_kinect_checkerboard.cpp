#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <turtlebot_kinect_arm_calibration/detect_calibration_pattern.h>

using namespace std;
using namespace Eigen;

//const std::string world_frame = "/ground_frame";
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
const double checkerboard_grid = 0.027;

class CalibrateKinectCheckerboard
{
    // Nodes and publishers/subscribers
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub_;
    ros::Publisher detector_pub_;
  
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
    
    // Have we calibrated the camera yet?
    bool calibrated;

public:
  CalibrateKinectCheckerboard()
    : nh_(), it_(nh_), calibrated(false)
  {
    pattern_detector_.setPattern(cv::Size(checkerboard_width, checkerboard_height), checkerboard_grid, CHESSBOARD);
    //convertIdealPointsToPointVector();
    
    transform_.translation().setZero();
    transform_.matrix().topLeftCorner<3, 3>() = Quaternionf().setIdentity().toRotationMatrix();
    
    // DEBUG
    pub_ = it_.advertise("calibration_pattern_out", 1);
    
    // Create subscriptions
    info_sub_ = nh_.subscribe(info_topic, 1, &CalibrateKinectCheckerboard::infoCallback, this);
    touch_transform.setOrigin( tf::Vector3(0, -0.016, -0.019) );
    touch_transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    
    // Also publishers
    detector_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("detector_cloud", 1);
    
    // Publish first gripper_touch transform
    tf_broadcaster_.sendTransform(tf::StampedTransform(touch_transform, ros::Time(0), tip_frame, touch_frame));
    
    // Create ideal points
    ideal_points_.push_back( pcl::PointXYZ(0, 0, 0) );
    ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, 0, 0) );
    ideal_points_.push_back( pcl::PointXYZ(0, (checkerboard_height-1)*checkerboard_grid, 0) );
    ideal_points_.push_back( pcl::PointXYZ((checkerboard_width-1)*checkerboard_grid, (checkerboard_height-1)*checkerboard_grid, 0) );
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
    // Send out gripper touch transform
    tf_broadcaster_.sendTransform(tf::StampedTransform(touch_transform, ros::Time(0), tip_frame, touch_frame));
  
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
      return;
      
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
  
  
  }
  

  void publishCloud(pcl::PointCloud<pcl::PointXYZ> detector_points, tf::Transform &transform, const std::string frame_id)
  {
    // Display to rviz
    pcl::PointCloud<pcl::PointXYZ> transformed_detector_points;
    
    
    pcl_ros::transformPointCloud(detector_points, transformed_detector_points, transform);
    
    transformed_detector_points.header.frame_id = frame_id;
    detector_pub_.publish(transformed_detector_points);
  }  
  
  void calibrate()
  {
    cout << "Is the checkerboard correct? " << endl;
    cout << "Move edge of gripper to point 1 in image and press Enter. " << endl;
    cout << "Move edge of gripper to point 2 in image and press Enter. " << endl;
    cout << "Move edge of gripper to point 3 in image and press Enter. " << endl;
    cout << "Move edge of gripper to point 4 in image and press Enter. " << endl;
    cout << "Does this transform seem right? " << endl;
    cout << "Run this in the command line: " << endl;
    
    
  
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

