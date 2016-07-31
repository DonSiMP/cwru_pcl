/**
 *  cwru_pcl_utils
 *  a ROS library providing useful PCL utility and example functions from CWRU
 *  
 *  Copyright 2015 CWRU Mobile Robotics Laboriatory
 *  Author: Wyatt Newman
 *  Contributors: Luc Bettaieb
 */

#include <cwru_pcl_utils/cwru_pcl_utils.h>

CwruPclUtils::CwruPclUtils(ros::NodeHandle* nodehandle):
  nh_(*nodehandle),
  pclKinect_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
  pclTransformed_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
  pclSelectedPoints_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
  pclTransformedSelectedPoints_ptr_(new pcl::PointCloud<pcl::PointXYZ>)
{
  initializeSubscribers();
  got_kinect_cloud_ = false;
  got_selected_points_ = false;
}

void CwruPclUtils::fit_points_to_plane(Eigen::MatrixXf points_mat, Eigen::Vector3f &plane_normal, double &plane_dist)
{
  ROS_INFO("[cwru_pcl_utils: fit_points_to_plane] Starting identification of plane from data: ");

  // first compute the centroid of the data:
  Eigen::Vector3f centroid;
  // here's a handy way to initialize data to all zeros; more variants exist
  centroid = Eigen::MatrixXf::Zero(3, 1);  // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
  // add all the points together:
  int npts = points_mat.cols();  // number of points = number of columns in matrix; check the size

  for (uint ipt = 0; ipt < npts; ipt++)
  {
    centroid += points_mat.col(ipt);  // add all the column vectors together
  }
  centroid /= npts;  // divide by the number of points to get the centroid
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Centroid found, removing it from all points.");

  // subtract this centroid from all points in points_mat:
  Eigen::MatrixXf points_offset_mat = points_mat;
  for (int ipt = 0; ipt < npts; ipt++)
  {
    points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid;
  }

  // compute the covariance matrix w/rt x,y,z:
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Computing covariance.");
  Eigen::Matrix3f CoVar;
  CoVar = points_offset_mat*(points_offset_mat.transpose());  // 3xN matrix times Nx3 matrix is 3x3
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Covariance found!");

  // here is a more complex object: a solver for eigenvalues/eigenvectors;
  // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Initializing eigenvalue solver with computed covariance.");
  Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

  Eigen::VectorXf evals;  // we'll extract the eigenvalues to here
  evals = es3f.eigenvalues().real();  // grab just the real parts
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Real eigenvalues determined.");

  // our solution should correspond to an e-val of zero, which will be the minimum eval
  // (all other evals for the covariance matrix will be >0)
  // however, the solution does not order the evals, so we'll have to find the one of interest ourselves
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Finding minimum eigenvalue.");
  double min_lambda = evals[0];  // initialize the hunt for min eval
  Eigen::Vector3cf complex_vec;  // here is a 3x1 vector of double-precision, complex numbers

  complex_vec = es3f.eigenvectors().col(0);  // Here's the first e-vec, corresponding to first e-val
  plane_normal = complex_vec.real();  // Strip off the real part

  double lambda_test;
  int i_normal = 0;
  // Loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
  for (int ivec = 1; ivec < 3; ivec++)
  {
    lambda_test = evals[ivec];
    if (lambda_test < min_lambda)
    {
      min_lambda = lambda_test;
      i_normal = ivec;  // this index is closer to index of min eval
      plane_normal = es3f.eigenvectors().col(ivec).real();
    }
  }

  // Found the minimum eigenvalue.
  ROS_DEBUG("[cwru_pcl_utils: fit_points_to_plane] Minimum eigenvalue found: %d", min_lambda);

  // at this point, we have the minimum eval in "min_lambda", and the plane normal
  // (corresponding evec) in "est_plane_normal"/
  // these correspond to the ith entry of i_normal
  plane_dist = plane_normal.dot(centroid);
  ROS_INFO("[cwru_pcl_utils: fit_points_to_plane] Plane distance found: %d", plane_dist);
}

void CwruPclUtils::fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                                       Eigen::Vector3f &plane_normal,
                                      double &plane_dist)
{
  Eigen::MatrixXf points_mat;
  Eigen::Vector3f cloud_pt;
  // populate points_mat from cloud data;

  int npts = input_cloud_ptr->points.size();
  points_mat.resize(3, npts);

  // somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud, with
  // conversions to/from Eigen
  for (int i = 0; i < npts; ++i)
  {
    cloud_pt = input_cloud_ptr->points[i].getVector3fMap();
    points_mat.col(i) = cloud_pt;
  }

  fit_points_to_plane(points_mat, plane_normal, plane_dist);
}

// this fnc operates on transformed selected points
void CwruPclUtils::fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal, double &plane_dist)
{
  fit_points_to_plane(pclTransformedSelectedPoints_ptr_, plane_normal, plane_dist);
}

Eigen::Affine3f CwruPclUtils::transformTFToEigen(const tf::Transform &t)
{
  Eigen::Affine3f e;
  // treat the Eigen::Affine as a 4x4 matrix:
  for (int i = 0; i < 3; i++)
  {
    e.matrix()(i, 3) = t.getOrigin()[i];  // copy the origin from tf to Eigen
    for (int j = 0; j < 3; j++)
    {
      e.matrix()(i, j) = t.getBasis()[i][j];  // and copy 3x3 rotation matrix
    }
  }
  // Fill in identity in last row
  for (int col = 0; col < 3; col++)
  {
    e.matrix()(3, col) = 0;
  }

  e.matrix()(3, 3) = 1;
  return e;
}

/**
 * here is a function that transforms a cloud of points into an alternative frame;
 * it assumes use of pclKinect_ptr_ from kinect sensor as input, to pclTransformed_ptr_ , the cloud in output frame
 * 
 * @param A [in] supply an Eigen::Affine3f, such that output_points = A*input_points
 */
void CwruPclUtils::transform_kinect_cloud(Eigen::Affine3f A)
{
  transform_cloud(A, pclKinect_ptr_, pclTransformed_ptr_);
  // pclTransformed_ptr_->header = pclKinect_ptr_->header;
  // pclTransformed_ptr_->is_dense = pclKinect_ptr_->is_dense;
  // pclTransformed_ptr_->width = pclKinect_ptr_->width;
  // pclTransformed_ptr_->height = pclKinect_ptr_->height;
  // int npts = pclKinect_ptr_->points.size();
  // std::cout << "transforming npts = " << npts << std::endl;
  // pclTransformed_ptr_->points.resize(npts);

  // // somewhat odd notation: getVector3fMap() reading OR WRITING points from/to a pointcloud,
  // // with conversions to/from Eigen
  // for (int i = 0; i < npts; ++i)
  // {
  //   pclTransformed_ptr_->points[i].getVector3fMap() = A * pclKinect_ptr_->points[i].getVector3fMap();
  // }
}

void CwruPclUtils::transform_selected_points_cloud(Eigen::Affine3f A)
{
  transform_cloud(A, pclSelectedPoints_ptr_, pclTransformedSelectedPoints_ptr_);
}

void CwruPclUtils::transform_cloud(Eigen::Affine3f A,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr)
{
  output_cloud_ptr->header = input_cloud_ptr->header;
  output_cloud_ptr->is_dense = input_cloud_ptr->is_dense;
  output_cloud_ptr->width = input_cloud_ptr->width;
  output_cloud_ptr->height = input_cloud_ptr->height;
  int npts = input_cloud_ptr->points.size();
  std::cout << "transforming npts = " << npts << std::endl;
  output_cloud_ptr->points.resize(npts);

  // somewhat odd notation: getVector3fMap() reading OR WRITING points
  // from/to a pointcloud, with conversions to/from Eigen
  for (int i = 0; i < npts; ++i)
  {
      output_cloud_ptr->points[i].getVector3fMap() = A * input_cloud_ptr->points[i].getVector3fMap();
  }
}

// member helper function to set up subscribers;
// note odd syntax: &ExampleRosClass::subscriberCallback is a pointer to a member function of ExampleRosClass
// "this" keyword is required, to refer to the current instance of ExampleRosClass
void CwruPclUtils::initializeSubscribers()
{
  ROS_INFO("Initializing Subscribers");

  pointcloud_subscriber_ = nh_.subscribe("/kinect/depth/points", 1, &CwruPclUtils::kinectCB, this);

  // subscribe to "selected_points", which is published by Rviz tool
  selected_points_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>("/selected_points",
                                                                        1,
                                                                        &CwruPclUtils::selectCB,
                                                                        this);
}

/**
 * callback fnc: receives transmissions of Kinect data; if got_kinect_cloud is false, 
 * copy current transmission to internal variable
 * @param cloud [in] messages received from Kinect
 */
void CwruPclUtils::kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    // std::cout << "callback from kinect pointcloud pub" << std::endl;
    // convert/copy the cloud only if desired
    if (!got_kinect_cloud_)
    {
      pcl::fromROSMsg(*cloud, *pclKinect_ptr_);
      ROS_INFO("kinectCB: got cloud with %i * %i points", (int)pclKinect_ptr_->width, (int)pclKinect_ptr_->height);
      got_kinect_cloud_ = true;  // cue to "main" that callback received and saved a pointcloud
    }
    // pcl::io::savePCDFileASCII("snapshot.pcd", *g_pclKinect);
    // ROS_INFO("saved PCD image consisting of %d data points to snapshot.pcd", (int)g_pclKinect->points.size());
}

// this callback wakes up when a new "selected Points" message arrives
void CwruPclUtils::selectCB(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  pcl::fromROSMsg(*cloud, *pclSelectedPoints_ptr_);
  ROS_INFO("RECEIVED NEW PATCH w/  %d * %d points", pclSelectedPoints_ptr_->width, pclSelectedPoints_ptr_->height);
  got_selected_points_ = true;
}
