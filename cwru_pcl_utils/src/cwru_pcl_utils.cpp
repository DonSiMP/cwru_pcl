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
  // ROS_INFO("starting identification of plane from data: ");
  // first compute the centroid of the data:
  Eigen::Vector3f centroid;
  // here's a handy way to initialize data to all zeros; more variants exist
  centroid = Eigen::MatrixXf::Zero(3, 1);  // see http://eigen.tuxfamily.org/dox/AsciiQuickReference.txt
  // add all the points together:
  int npts = points_mat.cols();  // number of points = number of columns in matrix; check the size

  for (int ipt = 0; ipt < npts; ipt++)
  {
    centroid += points_mat.col(ipt);  // add all the column vectors together
  }
  centroid /= npts;  // divide by the number of points to get the centroid
  // std::cout << "centroid: " << centroid.transpose() << std::endl;

  // subtract this centroid from all points in points_mat:
  Eigen::MatrixXf points_offset_mat = points_mat;
  for (int ipt = 0; ipt < npts; ipt++)
  {
    points_offset_mat.col(ipt) = points_offset_mat.col(ipt) - centroid;
  }

  // compute the covariance matrix w/rt x,y,z:
  Eigen::Matrix3f CoVar;
  CoVar = points_offset_mat*(points_offset_mat.transpose());  // 3xN matrix times Nx3 matrix is 3x3
  // std::cout << "covariance: " << std::endl;
  // std::cout << CoVar << std::endl;

  // here is a more complex object: a solver for eigenvalues/eigenvectors;
  // we will initialize it with our covariance matrix, which will induce computing eval/evec pairs
  Eigen::EigenSolver<Eigen::Matrix3f> es3f(CoVar);

  Eigen::VectorXf evals;  // we'll extract the eigenvalues to here
  // std::cout<<"size of evals: "<<es3d.eigenvalues().size()<<std::endl;
  // std::cout<<"rows,cols = "<<es3d.eigenvalues().rows()<<", "<<es3d.eigenvalues().cols()<<std::endl;
  // std::cout << "The eigenvalues of CoVar are:" << std::endl << es3d.eigenvalues().transpose() << std::endl;
  // std::cout<<"(these should be real numbers, and one of them should be zero)"<<std::endl;
  // std::cout << "The matrix of eigenvectors, V, is:" << std::endl;
  // std::cout<< es3d.eigenvectors() << std::endl << std::endl;
  // std::cout<< "(these should be real-valued vectors)"<<std::endl;

  // in general, the eigenvalues/eigenvectors can be complex numbers
  // however, since our matrix is self-adjoint (symmetric, positive semi-definite), we expect
  // real-valued evals/evecs;  we'll need to strip off the real parts of the solution

  evals = es3f.eigenvalues().real();  // grab just the real parts
  // std::cout << "real parts of evals: " << evals.transpose() << std::endl;

  // our solution should correspond to an e-val of zero, which will be the minimum eval
  // (all other evals for the covariance matrix will be >0)
  // however, the solution does not order the evals, so we'll have to find the one of interest ourselves

  double min_lambda = evals[0];  // initialize the hunt for min eval
  Eigen::Vector3cf complex_vec;  // here is a 3x1 vector of double-precision, complex numbers

  complex_vec = es3f.eigenvectors().col(0);  // here's the first e-vec, corresponding to first e-val
  // std::cout<<"complex_vec: "<<std::endl;
  // std::cout<<complex_vec<<std::endl;
  plane_normal = complex_vec.real();  // strip off the real part
  // std::cout<<"real part: "<<est_plane_normal.transpose()<<std::endl;
  // est_plane_normal = es3d.eigenvectors().col(0).real(); // evecs in columns

  double lambda_test;
  int i_normal = 0;
  // loop through "all" ("both", in this 3-D case) the rest of the solns, seeking min e-val
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
  // at this point, we have the minimum eval in "min_lambda", and the plane normal
  // (corresponding evec) in "est_plane_normal"/
  // these correspond to the ith entry of i_normal

  // std::cout << "min eval is " << min_lambda << ", corresponding to component " << i_normal << std::endl;
  // std::cout << "corresponding evec (est plane normal): " << est_plane_normal.transpose() << std::endl;
  // std::cout << "correct answer is: " << normal_vec.transpose() << std::endl;
  plane_dist = plane_normal.dot(centroid);

  // std::cout << "est plane distance from origin = " << est_dist << std::endl;
  // std::cout << "correct answer is: " << dist << std::endl;
  // std::cout << std::endl << std::endl;
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
