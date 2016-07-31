/**
 *  cwru_pcl_utils
 *  a ROS library providing useful PCL utility and example functions from CWRU
 *  
 *  Copyright 2015 CWRU Mobile Robotics Laboriatory
 *  Author: Wyatt Newman
 *  Contributors: Luc Bettaieb
 */

#ifndef CWRU_PCL_UTILS_H
#define CWRU_PCL_UTILS_H

// C++ Standard Library
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

class CwruPclUtils
{
public:
  CwruPclUtils(ros::NodeHandle *nodehandle);

  /**
  * provide an array of 3-D points (in columns), and this function will use and eigen-vector approach to find the best-fit plane
  * It returns the plane's normal vector and the plane's (signed) distance from the origin.
  * @param points_array input: points_array is a matrix of 3-D points to be plane-fitted; coordinates are in columns
  * @param plane_normal output: this function will compute components of the plane normal here
  * @param plane_dist output: scalar (signed) distance of the plane from the origin
  */
  void fit_points_to_plane(Eigen::MatrixXf points_array,
                           Eigen::Vector3f &plane_normal,
                           double &plane_dist);

  void fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                           Eigen::Vector3f &plane_normal,
                           double &plane_dist);

  void fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal,
                                         double &plane_dist);

  /**a utility fnc to convert tf::Transform type into an Eigen::Affine3f
   * Affine type "f" is needed for use with point clouds, since "floats" not "doubles" are more practical, considering
   * the expected (limited) resolution of the sensor, as well as the large size of point clouds
   * @param t  [in] provide a transform, e.g. per:
   *     g_tfListenerPtr->lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
   * @return an Eigen Affine object, A, such that point_in_new_frame = A*point_in_original_frame
   */    
  Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
  void transform_kinect_cloud(Eigen::Affine3f A);
  void transform_selected_points_cloud(Eigen::Affine3f A);

  void transform_cloud(Eigen::Affine3f A,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr);


  Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);

  void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud);

  void copy_cloud_xyzrgb_indices(PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                                 vector<int> &indices,
                                 PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);

  void copy_indexed_pts_to_output_cloud(vector<int> &indices,
                                        PointCloud<pcl::PointXYZRGB> &outputCloud);

  void get_gen_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> &outputCloud);

  void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                      double z_nom,
                      double z_eps,
                      vector<int> &indices);

  // as above, specifically for transformed kinect cloud:
  void find_coplanar_pts_z_height(double plane_height,
                                  double z_eps,
                                  vector<int> &indices);

  // find pts within +/- z_eps of z_height, AND within "radius" of "centroid"
  void filter_cloud_z(PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                      double z_nom,
                      double z_eps,
                      double radius,
                      Eigen::Vector3f centroid,
                      vector<int> &indices);

  // same as above, but specifically operates on transformed kinect cloud
  void filter_cloud_z(double z_nom, 
                      double z_eps,
                      double radius,
                      Eigen::Vector3f centroid,
                      vector<int> &indices);   

  Eigen::Vector3d find_avg_color();

  Eigen::Vector3d find_avg_color_selected_pts(vector<int> &indices);

  void find_indices_color_match(vector<int> &input_indices,
                                Eigen::Vector3d normalized_avg_color,
                                double color_match_thresh,
                                vector<int> &output_indices);

  void example_pcl_operation();

  void analyze_selected_points_color();

  inline Eigen::Vector3f get_centroid()
  {
    return centroid_;
  }

  inline Eigen::Vector3f get_major_axis()
  {
    return major_axis_;
  }

  inline void reset_got_kinect_cloud()
  {
    got_kinect_cloud_ = false;
  }

  inline void reset_got_selected_points()
  {
    got_selected_points_ = false;
  }

  inline bool got_kinect_cloud()
  {
    return got_kinect_cloud_;
  }

  inline bool got_selected_points()
  {
    return got_selected_points_;
  }

  inline void save_kinect_snapshot()
  {
    pcl::io::savePCDFileASCII("kinect_snapshot.pcd", *pclKinect_ptr_);
  }

  inline void save_kinect_clr_snapshot()
  {
    pcl::io::savePCDFileASCII("kinect_clr_snapshot.pcd", *pclKinect_clr_ptr_);
  }

  inline void save_transformed_kinect_snapshot()
  {
    pcl::io::savePCDFileASCII("xformed_kinect_snapshot.pcd", *pclTransformed_ptr_);
  }

  inline void get_indices(vector<int> &indices)
  {
    indices = indices_;
  }



private:
  ros::NodeHandle nh_;

  ros::Subscriber pointcloud_subscriber_;  // use this to subscribe to a pointcloud topic
  ros::Subscriber selected_points_subscriber_;  // this to subscribe to "selectedPoints" topic from Rviz

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_;  // (new PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedSelectedPoints_ptr_;

  bool got_kinect_cloud_;
  bool got_selected_points_;

  // we will define some helper methods to encapsulate the gory details of initializing
  // subscribers, publishers and services
  void initializeSubscribers();

  // TODO(enhancement)
  // void initializePublishers();
  // void initializeServices();

  void kinectCB(const sensor_msgs::PointCloud2ConstPtr &cloud);  // prototype for callback fnc
  void selectCB(const sensor_msgs::PointCloud2ConstPtr &cloud);  // callback for selected points

  // TODO(enhancement)
  // bool serviceCallback(example_srv::simple_bool_service_messageRequest& request,
  //                      example_srv::simple_bool_service_messageResponse& response);
};

#endif  // CWRU_PCL_UTILS_H
