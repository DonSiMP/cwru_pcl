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

// C++ Standard Libraries
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// ROS Messages
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

// Eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

class CwruPclUtils
{
public:
  /*
   * @brief
   *
   * @param
   */
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

  /*
   * @brief
   *
   * @param
   */
  Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);

  /*
   * @brief
   *
   * @param
   */
  Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ> &input_cloud);

  /*
   * @brief
   *
   * @param
   */
  void fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                           Eigen::Vector3f &plane_normal,
                           double &plane_dist);
  /*
   * @brief
   *
   * @param
   */
  // TODO(wsnewman) Was this deprecated?
  void fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal,
                                         double &plane_dist);

  /**
   * a utility fnc to convert tf::Transform type into an Eigen::Affine3f
   * Affine type "f" is needed for use with point clouds, since "floats" not "doubles" are more practical, considering
   * the expected (limited) resolution of the sensor, as well as the large size of point clouds
   * @param t  [in] provide a transform, e.g. per:
   *     g_tfListenerPtr->lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
   * @return an Eigen Affine object, A, such that point_in_new_frame = A*point_in_original_frame
   */    
  Eigen::Affine3f transformTFToEigen(const tf::Transform &t);

  /** 
   * function to create an Eigen-style Affine transform based on construction of a coordinate frame
   * placed on the surface of a plane
   */
  Eigen::Affine3f make_affine_from_plane_params(Eigen::Vector3f plane_normal, double plane_dist);

  /**
   * returns an Eigen::Affine transform to a coordinate frame constructed on a plane defined by
   * plane_parameters (normal_x, normal_y, normal_z, distance)
   * useful for transforming data to find planar surfaces with z-axis vertical
   */
  Eigen::Affine3f make_affine_from_plane_params(Eigen::Vector4f plane_parameters);

  /*
   * @brief
   *
   * @param
   */
  Eigen::Affine3f make_affine_from_plane_params(Eigen::Vector4f plane_parameters, Eigen::Vector3f centroid);

  /*
   * @brief
   *
   * @param
   */
  void transform_kinect_cloud(Eigen::Affine3f A);

  /*
   * @brief
   *
   * @param
   */
  void transform_selected_points_cloud(Eigen::Affine3f A);

  /*
   * @brief
   *
   * @param
   */
  void transform_cloud(Eigen::Affine3f A,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr);

  /*
   * @brief
   *
   * @param
   */
  // color version:
  void transform_cloud(Eigen::Affine3f A,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr);

  /*
   * @brief
   *
   * @param
   */
  inline void reset_got_kinect_cloud()
  {
    got_kinect_cloud_ = false;
  }

  /*
   * @brief
   *
   * @param
   */
  inline void reset_got_selected_points()
  {
    got_selected_points_ = false;
  }

  /*
   * @brief
   *
   * @param
   */
  inline bool got_kinect_cloud()
  {
    return got_kinect_cloud_;
  }

  /*
   * @brief
   *
   * @param
   */
  inline bool got_selected_points()
  {
    return got_selected_points_;
  }

  /*
   * @brief
   *
   * @param
   */
  inline void save_kinect_snapshot()
  {
    pcl::io::savePCDFileASCII("kinect_snapshot.pcd", *pclKinect_ptr_);
  }  // B/W

  /*
   * @brief
   *
   * @param
   */
  int read_pcd_file(std::string fname);

  /*
   * @brief
   *
   * @param
   */
  int read_clr_pcd_file(std::string fname);

  /*
   * @brief
   *
   * @param
   */
  // alternative "save" fnc: save as a colored pointcloud
  inline void save_kinect_clr_snapshot()
  {
    pcl::io::savePCDFileASCII("kinect_clr_snapshot.pcd", *pclKinect_clr_ptr_);
  }

  /*
   * @brief
   *
   * @param
   */
  inline int save_kinect_clr_snapshot_binary()
  {
    return pcl::io::savePCDFile("kinect_clr_snapshot_bin.pcd", *pclKinect_clr_ptr_, true);
  }

  /*
   * @brief
   *
   * @param
   */
  inline void save_transformed_kinect_snapshot()
  {
    pcl::io::savePCDFileASCII("xformed_kinect_snapshot.pcd", *pclTransformed_ptr_);
  }

  /*
   * @brief
   *
   * @param
   */
  void get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud);

  /*
   * @brief
   *
   * @param
   */
  void get_copy_selected_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloud);

  /*
   * @brief
   *
   * @param
   */
  void copy_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud);

  /*
   * @brief
   *
   * @param
   */
  void copy_cloud_xyzrgb_indices(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                                 std::vector<int> &indices,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud);

  /*
   * @brief
   *
   * @param
   */
  inline void get_indices(std::vector<int> &indices)
  {
    indices = indices_;
  }

  /*
   * @brief
   *
   * @param
   */
  // same as above, but assumes 
  void copy_indexed_pts_to_output_cloud(std::vector<int> &indices,
                                        pcl::PointCloud<pcl::PointXYZRGB> &outputCloud);

  /*
   * @brief
   *
   * @param
   */
  void get_gen_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> &outputCloud); 

  /*
   * @brief
   *
   * @param
   */
  void get_kinect_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud);

  /*
   * @brief
   *
   * @param
   */
  void get_kinect_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr);

  /*
   * @brief
   *
   * @param
   */
  void get_kinect_points(pcl::PointCloud<pcl::PointXYZRGB> &outputCloudPtr);

  /*
   * @brief
   *
   * @param
   */
  void get_kinect_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &outputCloud);

  /*
   * @brief
   *
   * @param
   */
  void get_selected_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &outputCloudPtr);

  /*
   * @brief
   *
   * @param
   */
  void get_selected_points(pcl::PointCloud<pcl::PointXYZ> &outputCloud);


  /*
   * @brief
   *
   * @param
   */
  void example_pcl_operation();

  /*
   * @brief
   *
   * @param
   */
  // operate on transformed Kinect data and identify point indices within +/-z_eps of specified height
  void filter_cloud_z(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                      double z_nom,
                      double z_eps,
                      std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  void filter_cloud_z(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                      double z_nom,
                      double z_eps,
                      std::vector<int> &indices);
  /*
   * @brief
   *
   * @param
   */   
  // as above, specifically for transformed kinect cloud:
  void find_coplanar_pts_z_height(double plane_height,
                                  double z_eps,
                                  std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  // find pts within +/- z_eps of z_height, AND within "radius" of "centroid"
  void filter_cloud_z(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                      double z_nom,
                      double z_eps,
                      double radius,
                      Eigen::Vector3f centroid,
                      std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  // same as above, but specifically operates on transformed kinect cloud
  void filter_cloud_z(double z_nom,
                      double z_eps,
                      double radius,
                      Eigen::Vector3f centroid,
                      std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  void box_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                  Eigen::Vector3f pt_min,
                  Eigen::Vector3f pt_max,
                  std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  void box_filter(Eigen::Vector3f pt_min,
                  Eigen::Vector3f pt_max,
                  std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  void analyze_selected_points_color();

  /*
   * @brief
   *
   * @param
   */
  inline Eigen::Vector3f get_centroid()
  {
    return centroid_;
  }

  /*
   * @brief
   *
   * @param
   */
  inline Eigen::Vector3f get_major_axis()
  {
    return major_axis_;
  }

  /*
   * @brief
   *
   * @param
   */
  Eigen::Vector3f get_patch_normal()
  {
    return patch_normal_;
  }

  /*
   * @brief
   *
   * @param
   */
  double get_patch_dist()
  {
    return patch_dist_;
  }

  /*
   * @brief
   *
   * @param
   */
  Eigen::Vector3d find_avg_color();

  /*
   * @brief
   *
   * @param
   */
  Eigen::Vector3d find_avg_color_selected_pts(std::vector<int> &indices);

  /*
   * @brief
   *
   * @param
   */
  void find_indices_color_match(std::vector<int> &input_indices,
                                Eigen::Vector3d normalized_avg_color,
                                double color_match_thresh,
                                std::vector<int> &output_indices);

private:
  /*
   * @brief
   */
  ros::NodeHandle nh_;
  // some objects to support subscriber, service, and publisher

  /*
   * @brief
   */
  ros::Subscriber pointcloud_subscriber_;  // use this to subscribe to a pointcloud topic

  /*
   * @brief
   */
  ros::Subscriber real_kinect_subscriber_;  // use this to subscribe to a physical kinect device

  /*
   * @brief
   */
  ros::Subscriber selected_points_subscriber_;  // this to subscribe to "selectedPoints" topic from Rviz

  /*
   * @brief
   */
  ros::Publisher  pointcloud_publisher_;

  /*
   * @brief
   */
  ros::Publisher patch_publisher_;

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_;  // pointer for color version of pointcloud

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclSelectedPtsClr_ptr_;  // pointer for color version of pointcloud

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_;  // (new pcl::PointCloud<pcl::PointXYZ>);

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_;

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr_;

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedSelectedPoints_ptr_;

  /*
   * @brief
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclGenPurposeCloud_ptr_;


  /*
   * @brief
   */
  bool got_kinect_cloud_;

  /*
   * @brief
   */
  bool got_selected_points_;

  /*
   * @brief
   */
  void initializeSubscribers();

  /*
   * @brief
   */
  void initializePublishers();

  /*
   * @brief
   *
   * @param
   */
  void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud); // prototype for callback fnc

  /*
   * @brief
   *
   * @param
   */
  void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud); // callback for selected points 

  /*
   * @brief
   */
  Eigen::Vector3f major_axis_, centroid_;

  /*
   * @brief
   */
  Eigen::Vector3d avg_color_;

  /*
   * @brief
   */
  Eigen::Vector3f patch_normal_;

  /*
   * @brief
   */
  double patch_dist_;

  /*
   * @brief
   */
  std::vector<int> indices_;  // put interesting indices here
};

#endif  // CWRU_PCL_UTILS_H
