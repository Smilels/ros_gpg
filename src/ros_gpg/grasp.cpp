#include <gpg/grasp.h>

const bool Grasp::MEASURE_TIME = false;
int Grasp::seed_ = 0;
Grasp::Grasp() : grasp_width_(0.0), label_(0.0, false, false)
{

}


Grasp::Grasp(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame,
  const FingerHand& finger_hand, double grasp_width)
: sample_(sample), grasp_width_(grasp_width)
{
  pose_.frame_ = frame;

  construct(finger_hand);
}


Grasp::Grasp(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame,
  const FingerHand& finger_hand)
: sample_(sample), grasp_width_(0.0)
{
  pose_.frame_ = frame;

  construct(finger_hand);
}


void Grasp::construct(const FingerHand& finger_hand)
{
  // finger positions and base/bottom and top/fingertip of grasp with respect to hand frame
  config_1d_.left_ = finger_hand.getLeft();
  config_1d_.right_ = finger_hand.getRight();
  config_1d_.top_ = finger_hand.getTop();
  config_1d_.bottom_ = finger_hand.getBottom();
  config_1d_.center_ = finger_hand.getCenter();

  // calculate grasp positions at the bottom/base and top of the hand and on the object surface
  calculateGraspPositions(finger_hand);

  // determine finger placement index that resulted in this grasp
  const Eigen::Array<bool, 1, Eigen::Dynamic>& indices = finger_hand.getHand();
  for (int i = 0; i < indices.size(); i++)
  {
    if (indices[i] == true)
    {
      finger_placement_index_ = i;
      break;
    }
  }

  label_.score_ = 0.0;
  label_.full_antipodal_ = false;
  label_.half_antipodal_ = false;
}


void Grasp::calculateGraspPositions(const FingerHand& finger_hand)
{
  // calculate grasp positions of hand middle on object surface, bottom/base and top/fingertip w.r.t. base frame
  Eigen::Vector3d pos_top, pos_bottom, pos_surface;
  pos_surface << finger_hand.getSurface(), finger_hand.getCenter(), 0.0;
  pos_bottom << getBottom(), finger_hand.getCenter(), 0.0;
  pos_top << getTop(), finger_hand.getCenter(), 0.0;
  pose_.surface_ = getFrame() * pos_surface + sample_;
  pose_.bottom_ = getFrame() * pos_bottom + sample_;
  pose_.top_ = getFrame() * pos_top + sample_;
}


void Grasp::writeHandsToFile(const std::string& filename, const std::vector<Grasp>& hands) const
{
  std::ofstream myfile;
  myfile.open (filename.c_str());

  for (int i = 0; i < hands.size(); i++)
  {
    std::cout << "Hand " << i << std::endl;
    print();

    myfile << vectorToString(hands[i].getGraspBottom()) << vectorToString(hands[i].getGraspSurface())
          << vectorToString(hands[i].getAxis()) << vectorToString(hands[i].getApproach())
          << vectorToString(hands[i].getBinormal()) << boost::lexical_cast<double>(hands[i].getGraspWidth()) << "\n";
  }

  myfile.close();
}


void Grasp::print() const
{
  std::cout << "approach: " << getApproach().transpose() << std::endl;
  std::cout << "binormal: " << getBinormal().transpose() << std::endl;
  std::cout << "axis: " << getAxis().transpose() << std::endl;
  std::cout << "grasp width: " << getGraspWidth() << std::endl;
  std::cout << "grasp surface: " << getGraspSurface().transpose() << std::endl;
  std::cout << "grasp bottom: " << getGraspBottom().transpose() << std::endl;
  std::cout << "grasp top: " << getGraspTop().transpose() << std::endl;
  std::cout << "score: " << getScore() << std::endl;
  std::cout << "half-antipodal: " << isHalfAntipodal() << std::endl;
  std::cout << "full-antipodal: " << isFullAntipodal() << std::endl;
  std::cout << "finger_hand:\n";
  std::cout << "  bottom: " << getBottom() << std::endl;
  std::cout << "  top: " << getTop() << std::endl;
}


std::string Grasp::vectorToString(const Eigen::VectorXd& v) const
{
  std::string s = "";
  for (int i = 0; i < v.rows(); i++)
  {
    s += boost::lexical_cast<std::string>(v(i)) + ",";
  }
  return s;
}

Eigen::Matrix3Xd Grasp::calculateShadow4(const PointList& point_list, double shadow_length) const
{
  double voxel_grid_size = 0.003; // voxel size for points that fill occluded region

  double num_shadow_points = floor(shadow_length / voxel_grid_size); // number of points along each shadow vector

  const int num_cams = point_list.getCamSource().rows();

  // Calculate the set of cameras which see the points.
  Eigen::VectorXi camera_set = point_list.getCamSource().rowwise().sum();

  // Calculate the center point of the point neighborhood.
  Eigen::Vector3d center = point_list.getPoints().rowwise().sum();
  center /= (double) point_list.size();

  // Stores the list of all bins of the voxelized, occluded points.
  std::vector< Vector3iSet > shadows;
  shadows.resize(num_cams, Vector3iSet(num_shadow_points * 10000));

  for (int i = 0; i < num_cams; i++)
  {
    if (camera_set(i) >= 1)
    {
      double t0_if = omp_get_wtime();

      // Calculate the unit vector that points from the camera position to the center of the point neighborhood.
      Eigen::Vector3d shadow_vec = center - point_list.getViewPoints().col(i);

      // Scale that vector by the shadow length.
      shadow_vec = shadow_length * shadow_vec / shadow_vec.norm();

      // Calculate occluded points.
      //      shadows[i] = calculateVoxelizedShadowVectorized4(point_list, shadow_vec, num_shadow_points, voxel_grid_size);
      calculateVoxelizedShadowVectorized(point_list.getPoints(), shadow_vec, num_shadow_points, voxel_grid_size, shadows[i]);
    }
  }

  // Only one camera view point.
  if (num_cams == 1)
  {
    // Convert voxels back to points.
    //    std::vector<Eigen::Vector3i> voxels(shadows[0].begin(), shadows[0].end());
    //    Eigen::Matrix3Xd shadow_out = shadowVoxelsToPoints(voxels, voxel_grid_size);
    //    return shadow_out;
    return shadowVoxelsToPoints(std::vector<Eigen::Vector3i>(shadows[0].begin(), shadows[0].end()), voxel_grid_size);
  }

  // Multiple camera view points: find the intersection of all sets of occluded points.
  double t0_intersection = omp_get_wtime();
  Vector3iSet bins_all = shadows[0];

  for (int i = 1; i < num_cams; i++)
  {
    if (camera_set(i) >= 1) // check that there are points seen by this camera
    {
      bins_all = intersection(bins_all, shadows[i]);
    }
  }
  if (MEASURE_TIME)
    std::cout << "intersection runtime: " << omp_get_wtime() - t0_intersection << "s\n";

  // Convert voxels back to points.
  std::vector<Eigen::Vector3i> voxels(bins_all.begin(), bins_all.end());
  Eigen::Matrix3Xd shadow_out = shadowVoxelsToPoints(voxels, voxel_grid_size);
  return shadow_out;
}


Eigen::Matrix3Xd Grasp::shadowVoxelsToPoints(const std::vector<Eigen::Vector3i>& voxels, double voxel_grid_size) const
{
  // Convert voxels back to points.
  double t0_voxels = omp_get_wtime();
  boost::mt19937 *rng = new boost::mt19937();
  rng->seed(time(NULL));
  boost::normal_distribution<> distribution(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(*rng, distribution);

  Eigen::Matrix3Xd shadow(3, voxels.size());

  for (int i = 0; i < voxels.size(); i++)
  {
    shadow.col(i) = voxels[i].cast<double>() * voxel_grid_size + Eigen::Vector3d::Ones() * generator()
        * voxel_grid_size * 0.3;
    //    shadow.col(i) = voxels[i].cast<double>() * voxel_grid_size;
    //    shadow.col(i)(0) += generator() * voxel_grid_size * 0.3;
    //    shadow.col(i)(1) += generator() * voxel_grid_size * 0.3;
    //    shadow.col(i)(2) += generator() * voxel_grid_size * 0.3;
  }
  if (MEASURE_TIME)
    std::cout << "voxels-to-points runtime: " << omp_get_wtime() - t0_voxels << "s\n";

  return shadow;
}


void Grasp::calculateVoxelizedShadowVectorized(const Eigen::Matrix3Xd& points,
  const Eigen::Vector3d& shadow_vec, int num_shadow_points, double voxel_grid_size, Vector3iSet& shadow_set) const
{
  double t0_set = omp_get_wtime();
  const int n = points.cols() * num_shadow_points;
  const double voxel_grid_size_mult = 1.0 / voxel_grid_size;
  const double max = 1.0 / 32767.0;
  //  Eigen::Vector3d w;

  for(int i = 0; i < n; i++)
  {
    const int pt_idx = i / num_shadow_points;
    //    const Eigen::Vector3d w = (points.col(pt_idx) + ((double) fastrand() * max) * shadow_vec) * voxel_grid_size_mult;
    shadow_set.insert(((points.col(pt_idx) + ((double) fastrand() * max) * shadow_vec) * voxel_grid_size_mult).cast<int>());
  }

  if (MEASURE_TIME)
    printf("Shadow (1 camera) calculation. Runtime: %.3f, #points: %d, num_shadow_points: %d, #shadow: %d, max #shadow: %d\n",
      omp_get_wtime() - t0_set, (int) points.cols(), num_shadow_points, (int) shadow_set.size(), n);
  //    std::cout << "Calculated shadow for 1 camera. Runtime: " << omp_get_wtime() - t0_set << ", #points: " << n << "\n";
}

inline int Grasp::fastrand() const
{
  seed_ = (214013*seed_+2531011);
  return (seed_>>16)&0x7FFF;
}


Vector3iSet Grasp::intersection(const Vector3iSet& set1, const Vector3iSet& set2) const
{
  if (set2.size() < set1.size())
  {
    return intersection(set2, set1);
  }

  Vector3iSet set_out(set1.size());

  for (Vector3iSet::const_iterator it = set1.begin(); it != set1.end(); it++)
  {
    if (set2.find(*it) != set2.end())
    {
      set_out.insert(*it);
    }
  }

  return set_out;
}
