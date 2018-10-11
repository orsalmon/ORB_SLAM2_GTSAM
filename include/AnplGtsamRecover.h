//
// Created by Or Salmon on 15/08/18.
//

#ifndef ORB_SLAM2_ANPL_GTSAM_RECOVER_H
#define ORB_SLAM2_ANPL_GTSAM_RECOVER_H

#include <mutex>

#include "Converter.h"

// For keys representation
#include <gtsam/inference/Symbol.h>

// For keyframes pose
#include <gtsam/geometry/StereoCamera.h>

// For factors between keyframes
#include <gtsam/slam/BetweenFactor.h>

// For prior knowledge
#include <gtsam/slam/PriorFactor.h>

// For landmarks position
#include <gtsam/geometry/Point3.h>

//// Mono ////
// For landmarks coordinates
#include <gtsam/geometry/Point2.h>
// For factors between keyframe and landmarks
#include <gtsam/slam/ProjectionFactor.h>

//// Stereo ////
// For landmarks coordinates
#include <gtsam/geometry/StereoPoint2.h>
// For factors between keyframe and landmarks
#include <gtsam/slam/StereoFactor.h>

// Factor Container
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
// Values Container
#include <gtsam/nonlinear/Values.h>

// Serialization
#include <gtsam/base/serialization.h>

namespace ORB_SLAM2 {
class KeyFrame;
class MapPoint;

class AnplGtsamRecover {
  friend class Optimizer;

 public:
  AnplGtsamRecover();
  ~AnplGtsamRecover();
  std::tuple<bool,
             boost::optional<bool>,
             boost::optional<std::string>,
             boost::optional<std::string>,
             boost::optional<std::vector<gtsam::Key>>> checkForNewData();

 protected:
  void addKeyFrame(KeyFrame *pKF, bool initiate_with_pose);
  void addLandmark(MapPoint *pMP, bool initiate_with_pose);
  void addMonoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 2, 1> &obs);
  void addStereoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 3, 1> &obs);
  void setKeyFramePose(KeyFrame *pKF, g2o::SE3Quat pose);
  void setLandmarkPose(MapPoint *pMP, g2o::Vector3d position);
  void start(bool is_full_BA);
  void finish();

 private:
  gtsam::NonlinearFactorGraph *graph_;
  gtsam::Values *values_;
  gtsam::Cal3_S2Stereo::shared_ptr cam_params_stereo_;
  gtsam::Cal3_S2::shared_ptr cam_params_mono_;
  gtsam::noiseModel::Diagonal::shared_ptr cam_pose_prior_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr landmark_position_prior_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr mono_observation_noise_model_;
  gtsam::noiseModel::Diagonal::shared_ptr stereo_observation_noise_model_;
  bool is_cam_params_initialized_ = false;
  bool new_optimized_data_ = false;
  bool is_full_BA_data_ = false;
  std::vector<gtsam::Key> del_keyframes_;

  std::string serialized_graph_, serialized_values_;

  std::mutex mutex_;
};
}

#endif //ORB_SLAM2_ANPL_GTSAM_RECOVER_H
