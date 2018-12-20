//
// Created by Or Salmon on 15/08/18.
//

#ifndef ORB_SLAM2_GTSAM_TRANSFORMER_H
#define ORB_SLAM2_GTSAM_TRANSFORMER_H

#include <mutex>
#include <set>
#include <algorithm>
#include <queue>

#include "Converter.h"

// For keys representation
#include <gtsam/inference/Symbol.h>

// For keyframes pose
#include <gtsam/geometry/StereoCamera.h>

// For between factors
#include <gtsam/slam/BetweenFactor.h>

// For landmarks position
#include <gtsam/geometry/Point3.h>

// For first keyframe pose
//#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/PriorFactor.h>

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

namespace ORB_SLAM2 {
class KeyFrame;
class MapPoint;

class GtsamTransformer {
  friend class Optimizer;

  enum FactorType {
    PRIOR,
    BETWEEN,
    MONO,
    STEREO
  };

 public:
  GtsamTransformer();

  /**
   * Returns tuple contains:
   * 1. Boolean indicates if there is a new data or not
   * 2. Optional Boolean indicates if the data is incremental update
   * 3. Optional string contains graph of the added factors since the last call to the function (serialized)
   * 4. Optional vector contains the pair's key (keyframe-landmark) of the factors had removed since the last call to the function
   * 5. Optional set contains the keys of the added states (keyframes/landmarks) since the last call to the function
   * 6. Optional set contains the keys of the removed states (keyframes/landmarks) since the last call to the function
   * 7. Optional GTSAM values object contains the values of the added states since the last call to the function (serialized)
   * 8. Optional tuple of the most recent keyframe symbol (serialized), its timestamp, and its Pose3 (serialized)
   */
  std::tuple<bool,
             boost::optional<bool>,
             boost::optional<std::string>,
             boost::optional<std::vector<size_t>>,
             boost::optional<std::set<gtsam::Key>>,
             boost::optional<std::set<gtsam::Key>>,
             boost::optional<std::string>,
             boost::optional<std::tuple<std::string, double, std::string>>> checkForNewData();

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
  bool start();
  void finish();

 private:
  void updateActiveSets();
  void exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                         std::vector<std::pair<gtsam::Key, gtsam::Key>> &output);
  void exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                           std::vector<std::pair<std::string, FactorType>> &output);
  std::string setToString(const std::set<gtsam::Key> &set) const;
  gtsam::NonlinearFactorGraph createFactorGraph(std::vector<std::pair<std::string, FactorType>> ser_factors_vec);
  gtsam::NonlinearFactorGraph createFactorGraph(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> ser_factors_map);
  std::vector<size_t> createDeletedFactorsIndicesVec(std::vector<std::pair<gtsam::Key, gtsam::Key>> &del_factors);
  // Private implementation of std::set_difference
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> getDifferenceSet(std::map<std::pair<gtsam::Key, gtsam::Key>,
                                                                                                            std::pair<std::string,
                                                                                                                      FactorType>> &set_A,
                                                                                                   std::map<std::pair<gtsam::Key, gtsam::Key>,
                                                                                                            std::pair<std::string,
                                                                                                                      FactorType>> &set_B);
  // Private implementation of std::set_intersection
  std::vector<std::pair<gtsam::Key, gtsam::Key>> getIntersectionVec(std::map<std::pair<gtsam::Key, gtsam::Key>,
                                                                             std::pair<std::string, FactorType>> &set_A,
                                                                    std::vector<std::pair<gtsam::Key, gtsam::Key>> &vec_B);


  std::queue<std::tuple<bool,
                        bool,
                        std::string,
                        std::vector<size_t>,
                        std::set<gtsam::Key>,
                        std::set<gtsam::Key>,
                        std::string,
                        std::tuple<std::string, double, std::string>>> ready_data_queue_;

  gtsam::Values values_, add_values_;
  gtsam::Cal3_S2Stereo::shared_ptr cam_params_stereo_;
  gtsam::Cal3_S2::shared_ptr cam_params_mono_;
  gtsam::noiseModel::Diagonal::shared_ptr between_factors_prior_;
  bool is_cam_params_initialized_ = false;
  std::vector<std::pair<std::string, FactorType>> add_factors_;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> del_factors_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> active_factors_, session_factors_;
  std::set<gtsam::Key> active_states_, del_states_, add_states_, session_states_;
  std::tuple<std::string, double, std::string> recent_kf_;
  std::map<std::pair<gtsam::Key, gtsam::Key>, size_t> factor_indecies_dict_;
  size_t current_index_ = 0;

  std::mutex mutex_;

  std::shared_ptr<spdlog::logger> logger_;
};
}

#endif //ORB_SLAM2_GTSAM_TRANSFORMER_H
