//
// Created by Or Salmon on 15/08/18.
//

#ifndef ORB_SLAM2_ANPL_GTSAM_RECOVER_H
#define ORB_SLAM2_ANPL_GTSAM_RECOVER_H

#include <mutex>
#include <set>
#include <algorithm>

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

// logger
#include <spdlog/spdlog.h>
#include <spdlog/sinks/file_sinks.h>

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
             boost::optional<std::set<unsigned long>>,
             boost::optional<std::set<unsigned long>>,
             boost::optional<std::set<unsigned long>>,
             boost::optional<std::set<unsigned long>>> checkForNewData();

 protected:
  void addKeyFrame(KeyFrame *pKF, bool fixed_pose);
  void addLandmark(MapPoint *pMP);
  void addMonoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 2, 1> &obs, const float inv_sigma_2);
  void addStereoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 3, 1> &obs, const float inv_sigma_2);
  void setKeyFramePose(KeyFrame *pKF, g2o::SE3Quat pose);
  void setLandmarkPose(MapPoint *pMP, g2o::Vector3d position);
  void handleKeyframe(KeyFrame *pKF, bool is_bad);
  void handleLandmark(MapPoint *pMP, bool is_bad);
  void updateActiveSets();
  void start(bool is_full_BA);
  void finish();

 private:
  std::string setToString(const std::set<unsigned long> &) const;

  gtsam::NonlinearFactorGraph *graph_ = nullptr;
  gtsam::Values *values_ = nullptr;
  gtsam::Cal3_S2Stereo::shared_ptr cam_params_stereo_;
  gtsam::Cal3_S2::shared_ptr cam_params_mono_;
  const double PRIOR_SIGMA_2;
  bool is_cam_params_initialized_ = false;
  bool new_optimized_data_ = false;
  bool is_full_BA_data_ = false;
  std::set<unsigned long> active_keyframes_, del_keyframes_, add_keyframes_, session_keyframes_;
  std::set<unsigned long> active_landmarks_, del_landmarks_, add_landmarks_, session_landmarks_;


  std::string serialized_graph_, serialized_values_;

  std::mutex mutex_;

  std::shared_ptr<spdlog::logger> logger_;
};
}

#endif //ORB_SLAM2_ANPL_GTSAM_RECOVER_H
