//
// Created by Or Salmon on 15/08/18.
//

#ifndef ORB_SLAM2_GTSAM_TRANSFORMER_H
#define ORB_SLAM2_GTSAM_TRANSFORMER_H

#include <mutex>
#include <set>
#include <algorithm>

#include "Converter.h"

// For keys representation
#include <gtsam/inference/Symbol.h>

// For keyframes pose
#include <gtsam/geometry/StereoCamera.h>

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
#include <gtsam/nonlinear/NonlinearFactor.h>
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

class GtsamTransformer {
  friend class Optimizer;

 public:
  GtsamTransformer();

  /**
   * Returns tuple contains:
   * 1. Boolean indicates if there is a new data or not
   * 2. Optional Boolean indicates if the data contains the whole graph (after global BA)
   * 3. Optional vector contains the added factors since the last call to the function (serialized)
   * 4. Optional vector contains the pair's key (keyframe-landmark) of the factors had removed since the last call to the function
   * 5. Optional set contains the keys of the added states (keyframes/landmarks) since the last call to the function
   * 6. Optional set contains the keys of the removed states (keyframes/landmarks) since the last call to the function
   * 7. Optional GTSAM values object contains the whole graph states with updated value
   */
  std::tuple<bool,
             boost::optional<bool>,
             boost::optional<std::vector<std::string>>,
             boost::optional<std::vector<std::pair<gtsam::Key, gtsam::Key>>>,
             boost::optional<std::set<gtsam::Key>>,
             boost::optional<std::set<gtsam::Key>>,
             boost::optional<gtsam::Values>> checkForNewData();

 protected:
  void addKeyFrame(KeyFrame *pKF);
  void addLandmark(MapPoint *pMP);
  void addMonoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 2, 1> &obs, const float inv_sigma_2);
  void addStereoMeasurement(KeyFrame *pKF, MapPoint *pMP, Eigen::Matrix<double, 3, 1> &obs, const float inv_sigma_2);
  void setKeyFramePose(KeyFrame *pKF, g2o::SE3Quat pose);
  void setLandmarkPose(MapPoint *pMP, g2o::Vector3d position);
  void handleKeyframe(KeyFrame *pKF, bool is_bad);
  void handleLandmark(MapPoint *pMP, bool is_bad);
  void removeFactor(KeyFrame *pKF, MapPoint *pMP);
  void start(bool is_full_BA);
  void finish();

 private:
  void updateActiveSets();
  void exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> &map, std::vector<std::pair<gtsam::Key, gtsam::Key>> &output);
  void exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> &map, std::vector<std::string> &output);
  std::string setToString(const std::set<gtsam::Key> &set) const;

  gtsam::Values values_;
  gtsam::Cal3_S2Stereo::shared_ptr cam_params_stereo_;
  gtsam::Cal3_S2::shared_ptr cam_params_mono_;
  bool is_cam_params_initialized_ = false;
  bool new_optimized_data_ = false;
  bool is_full_BA_data_ = false;
  std::vector<std::string> add_factors_;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> del_factors_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> active_factors_, session_factors_;
  std::set<gtsam::Key> active_states_, del_states_, add_states_, session_states_;

  std::mutex mutex_;

  std::shared_ptr<spdlog::logger> logger_;
};
}

#endif //ORB_SLAM2_GTSAM_TRANSFORMER_H
