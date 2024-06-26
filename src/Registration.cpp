#include "Registration.h"


//Point 1/5
struct PointDistance
{ 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // This class should include an auto-differentiable cost function. 
  // To rotate a point given an axis-angle rotation, use
  // the Ceres function:
  // AngleAxisRotatePoint(...) (see ceres/rotation.h)
  // Similarly to the Bundle Adjustment case initialize the struct variables with the source and  the target point.
  // You have to optimize only the 6-dimensional array (rx, ry, rz, tx ,ty, tz).
  // WARNING: When dealing with the AutoDiffCostFunction template parameters,
  // pay attention to the order of the template parameters
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  PointDistance(Eigen::Vector3d source, Eigen::Vector3d target ) : source(source), target(target) {}

  template<typename T>
  bool operator()(const T* const transformation, T* residual) const
  {
    //Source data
    Eigen::Matrix<T,3,1> source_point;
    source_point << T(source[0]), T(source[1]), T(source[2]);

    //Translation data
    T tx = transformation[3];
    T ty = transformation[4];
    T tz = transformation[5];

    //Rotate and traslate (our final point)
    Eigen::Matrix<T,3,1> rotated_point;
    ceres::AngleAxisRotatePoint(transformation, source_point.data(), rotated_point.data());
    rotated_point[0] += tx;
    rotated_point[1] += ty;
    rotated_point[2] += tz;

    //Compute residual (difference between our data and target)
    residual[0] = rotated_point.x() - target.x();
    residual[1] = rotated_point.y() - target.y();
    residual[2] = rotated_point.z() - target.z();

    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d source, const Eigen::Vector3d target)
  {
    return (new ceres::AutoDiffCostFunction<PointDistance,3,6>(new PointDistance(source, target)));
  }

  Eigen::Vector3d source;
  Eigen::Vector3d target;

};


Registration::Registration(std::string cloud_source_filename, std::string cloud_target_filename)
{
  open3d::io::ReadPointCloud(cloud_source_filename, source_ );
  open3d::io::ReadPointCloud(cloud_target_filename, target_ );
  Eigen::Vector3d gray_color;
  source_for_icp_ = source_;
}


Registration::Registration(open3d::geometry::PointCloud cloud_source, open3d::geometry::PointCloud cloud_target)
{
  source_ = cloud_source;
  target_ = cloud_target;
  source_for_icp_ = source_;
}


void Registration::draw_registration_result()
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  //different color
  Eigen::Vector3d color_s;
  Eigen::Vector3d color_t;
  color_s<<1, 0.706, 0;
  color_t<<0, 0.651, 0.929;

  target_clone.PaintUniformColor(color_t);
  source_clone.PaintUniformColor(color_s);
  source_clone.Transform(transformation_);

  auto src_pointer =  std::make_shared<open3d::geometry::PointCloud>(source_clone);
  auto target_pointer =  std::make_shared<open3d::geometry::PointCloud>(target_clone);
  open3d::visualization::DrawGeometries({src_pointer, target_pointer});
  return;
}


//Point 5/5
void Registration::execute_icp_registration(double threshold, int max_iteration, double relative_rmse, std::string mode)
{ 
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //ICP main loop
  //Check convergence criteria and the current iteration.
  //If mode=="svd" use get_svd_icp_transformation if mode=="lm" use get_lm_icp_transformation.
  //Remember to update transformation_ class variable, you can use source_for_icp_ to store transformed 3d points.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  double rmse = 0.0;

  for(int i=0;i<max_iteration;++i)
  {
    //Get the value from find_closest_point function
    std::tuple<std::vector<size_t>, std::vector<size_t>, double> closest_point = find_closest_point(threshold);
    std::vector<size_t> source_indices = std::get<0>(closest_point);
    std::vector<size_t> target_indices = std::get<1>(closest_point);
    double current_rmse = std::get<2>(closest_point);

    //Check the convergence criteria and update the rmse
    if(std::abs(current_rmse - rmse) < relative_rmse) {return;}
    rmse = current_rmse;

    //Creation of the transformations used to compute the final one
    Eigen::Matrix4d previous_transformation, current_transformation, final_transformation;
    previous_transformation = get_transformation();
    current_transformation = Eigen::Matrix4d::Identity();
    final_transformation = Eigen::Matrix4d::Identity();

    //Set the current_transformation according to the chosen mode
    if(mode == "svd")
    {
      current_transformation = get_svd_icp_transformation(source_indices,target_indices);
    }
    else if(mode == "lm")
    {
      current_transformation = get_lm_icp_registration(source_indices,target_indices);
    }

    //Assign value to the final_transformation
    final_transformation.block<3,3>(0,0) = current_transformation.block<3,3>(0,0) * previous_transformation.block<3,3>(0,0);
    final_transformation.block<3,1>(0,3) = current_transformation.block<3,3>(0,0) * previous_transformation.block<3,1>(0,3) + current_transformation.block<3,1>(0,3);

    //Update transformation_ class variable
    set_transformation(final_transformation);

    //store transformed 3D points
    source_for_icp_.Transform(current_transformation);

  }

  return;
}

//Point 2/5
std::tuple<std::vector<size_t>, std::vector<size_t>, double> Registration::find_closest_point(double threshold)
{ ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Find source and target indices: for each source point find the closest one in the target and discard if their 
  //distance is bigger than threshold
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  std::vector<size_t> target_indices;
  std::vector<size_t> source_indices;
  Eigen::Vector3d source_point;
  double rmse;
  double mse;

  //Creation of kdTree to speed up the search in 3D space
  open3d::geometry::KDTreeFlann target_kd_tree(target_);
  
  //SearchKNN needs: query, knn, indices (1), distance^2
  //Since we are looking for the closest point this two vectors contain only one element (knn = 1)
  std::vector<int> idx(1);
  std::vector<double> dist2(1);

  //Clone the source point (for icp)
  open3d::geometry::PointCloud source_clone = source_for_icp_;

  //Search the closest point and save its informations
  int points_number = source_clone.points_.size();
  for(size_t i = 0; i < points_number; ++i)
  {
    source_point = source_clone.points_[i];
    target_kd_tree.SearchKNN(source_point,1,idx,dist2);

    //Check the threshold
    if(sqrt(dist2[0]) <= threshold)
    {
      source_indices.push_back(i);
      target_indices.push_back(idx[0]);
      mse = mse * i/(i+1) + dist2[0]/(i+1);
    }
  }

  rmse = sqrt(mse); 

  return {source_indices, target_indices, rmse};
}

//Point 3/5
Eigen::Matrix4d Registration::get_svd_icp_transformation(std::vector<size_t> source_indices, std::vector<size_t> target_indices){
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Find point clouds centroids and subtract them. 
  //Use SVD (Eigen::JacobiSVD<Eigen::MatrixXd>) to find best rotation and translation matrix.
  //Use source_indices and target_indices to extract point to compute the 3x3 matrix to be decomposed.
  //Remember to manage the special reflection case.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(4,4);

  //Clone the source point (for icp)
  open3d::geometry::PointCloud source_clone = source_for_icp_;

  //Compute centroid:
  //ComputeMeanAndCovariance returns a tupla with the mean(the centroids) and the covariance matrix
  //Using get<0> I obtain only the mean value, what i need
  Eigen::Vector3d source_centroid, target_centroid;
  source_centroid = std::get<0>(source_clone.ComputeMeanAndCovariance());
  target_centroid = std::get<0>(target_.ComputeMeanAndCovariance());

  //Subtract the centroid and calculate the W matrix using the theorem showed on lecture 14
  Eigen::Matrix3d W;
  int source_size = source_indices.size();
  for(int i = 0; i<source_size; ++i)
  {
    Eigen::Vector3d d = source_clone.points_[source_indices[i]] - source_centroid;
    Eigen::Vector3d m = target_.points_[target_indices[i]] - target_centroid;
    W = W + m * d.transpose();
  }

  //SVD
  Eigen::JacobiSVD<Eigen::MatrixXd> SVD(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

  //Compute rotation
  Eigen::Matrix3d rotation = SVD.matrixU() * SVD.matrixV().transpose();

  //Special case: reflection
  //I have to check the value of det(UV^T) = det(U) * det(V^T) = det(U) * det(V)
  if(SVD.matrixU().determinant() * SVD.matrixV().determinant() == -1)
  {
    Eigen::DiagonalMatrix<double,3> diag;
    diag.diagonal() << 1.0, 1.0, -1.0;
    //R = U * diag(1,1,-1) * V^T
    rotation = SVD.matrixU() * diag * SVD.matrixV().transpose();
  }

  //Compute translation
  Eigen::Vector3d translation = target_centroid - (rotation * source_centroid);

  //Assign value R,t to transformation
  transformation.block<3,3>(0,0) = rotation;
  transformation.block<3,1>(0,3) = translation;

  return transformation;
}

//Point 4/5
Eigen::Matrix4d Registration::get_lm_icp_registration(std::vector<size_t> source_indices, std::vector<size_t> target_indices)
{
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Use LM (Ceres) to find best rotation and translation matrix. 
  //Remember to convert the euler angles in a rotation matrix, store it coupled with the final translation on:
  //Eigen::Matrix4d transformation.
  //The first three elements of std::vector<double> transformation_arr represent the euler angles, the last ones
  //the translation.
  //use source_indices and target_indices to extract point to compute the matrix to be decomposed.
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity(4,4);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  options.num_threads = 4;
  options.max_num_iterations = 100;
  std::vector<double> transformation_arr(6, 0.0);
  int num_points = source_indices.size();

  //Clone the source point (for icp)
  open3d::geometry::PointCloud source_clone = source_for_icp_;

  //I need for ceres (Struct variable)
  Eigen::Vector3d source;
  Eigen::Vector3d target;

  //Ceres problem and summary (used in solve())
  ceres::Problem problem; 
  ceres::Solver::Summary summary;

  // For each point....
  for( int i = 0; i < num_points; i++ )
  {
    source = source_clone.points_[source_indices[i]];
    target = target_.points_[target_indices[i]];

    ceres::CostFunction* cost_function = PointDistance::Create(source,target);
    problem.AddResidualBlock(cost_function,nullptr,transformation_arr.data());
  }

  //Solve the problem
  Solve(options, &problem, &summary);

  //Obtain the rotation data (euler formulation) and convert to rotation matrix formulation
  double roll,pitch,yaw;
  roll = transformation_arr[0];
  pitch = transformation_arr[1];
  yaw = transformation_arr[2];

  Eigen::AngleAxisd roll_angle {roll, Eigen::Vector3d::UnitX()};
  Eigen::AngleAxisd pitch_angle {pitch, Eigen::Vector3d::UnitY()};
  Eigen::AngleAxisd yaw_angle {yaw, Eigen::Vector3d::UnitZ()};

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  rotation = yaw_angle * pitch_angle * roll_angle;

  //Obtain the translation vector from tranformation_arr
  Eigen::Vector3d translation{transformation_arr[3],transformation_arr[4],transformation_arr[5]};

  //Assign value rotation,translation to transformation
  transformation.block<3,3>(0,0) = rotation;
  transformation.block<3,1>(0,3) = translation;

  return transformation;
}


void Registration::set_transformation(Eigen::Matrix4d init_transformation)
{
  transformation_=init_transformation;
}


Eigen::Matrix4d  Registration::get_transformation()
{
  return transformation_;
}

double Registration::compute_rmse()
{
  open3d::geometry::KDTreeFlann target_kd_tree(target_);
  open3d::geometry::PointCloud source_clone = source_;
  source_clone.Transform(transformation_);
  int num_source_points  = source_clone.points_.size();
  Eigen::Vector3d source_point;
  std::vector<int> idx(1);
  std::vector<double> dist2(1);
  double mse;
  for(size_t i=0; i < num_source_points; ++i) {
    source_point = source_clone.points_[i];
    target_kd_tree.SearchKNN(source_point, 1, idx, dist2);
    mse = mse * i/(i+1) + dist2[0]/(i+1);
  }
  return sqrt(mse);
}

void Registration::write_tranformation_matrix(std::string filename)
{
  std::ofstream outfile (filename);
  if (outfile.is_open())
  {
    outfile << transformation_;
    outfile.close();
  }
}

void Registration::save_merged_cloud(std::string filename)
{
  //clone input
  open3d::geometry::PointCloud source_clone = source_;
  open3d::geometry::PointCloud target_clone = target_;

  source_clone.Transform(transformation_);
  open3d::geometry::PointCloud merged = target_clone+source_clone;
  open3d::io::WritePointCloud(filename, merged );
}


