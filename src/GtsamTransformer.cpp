//
// Created by Or Salmon on 15/08/18.
//

#include "GtsamTransformer.h"

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Optimizer.h"

#include "GtsamSerializationHelper.h"

//#define DEBUG

namespace ORB_SLAM2 {
GtsamTransformer::GtsamTransformer() {
  logger_ = spdlog::rotating_logger_st("GtsamTransformer",
                                       "GtsamTransformer",
                                       1048576 * 50,
                                       3);
  logger_->set_level(spdlog::level::debug);
  logger_->info("CTOR - GtsamTransformer instance created");
}

void GtsamTransformer::addKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
#ifdef DEBUG
  logger_->debug("addKeyFrame - pKF->mnId: {}", pKF->mnId);
#endif
  // Create unique symbol
  gtsam::Symbol sym('x', pKF->mnId);
  // Create camera parameters
  if (!is_cam_params_initialized_) {
    cam_params_stereo_.reset(new gtsam::Cal3_S2Stereo(pKF->fx, pKF->fy, 0.0, pKF->cx, pKF->cy, pKF->mb));
    cam_params_mono_.reset(new gtsam::Cal3_S2(cam_params_stereo_->calibration()));
    is_cam_params_initialized_ = true;
  }
  // Create camera pose3
  cv::Mat T_cv = pKF->GetPose();
  gtsam::Matrix4 T_gtsam;
  T_gtsam << T_cv.at<double>(0, 0), T_cv.at<double>(0, 1), T_cv.at<double>(0, 2), T_cv.at<double>(0, 3)
      , T_cv.at<double>(1, 0), T_cv.at<double>(1, 1), T_cv.at<double>(1, 2), T_cv.at<double>(1, 3)
      , T_cv.at<double>(2, 0), T_cv.at<double>(2, 1), T_cv.at<double>(2, 2), T_cv.at<double>(2, 3)
      , T_cv.at<double>(3, 0), T_cv.at<double>(3, 1), T_cv.at<double>(3, 2), T_cv.at<double>(3, 3);
  gtsam::Pose3 left_cam_pose(T_gtsam);
  gtsam::StereoCamera stereo_cam(left_cam_pose, cam_params_stereo_);

  if (values_.exists(sym.key()))
    values_.update(sym.key(), stereo_cam);
  else
    values_.insert(sym.key(), stereo_cam);
}

void GtsamTransformer::addLandmark(ORB_SLAM2::MapPoint *pMP) {
#ifdef DEBUG
  logger_->debug("addLandmark - pMP->mnId: {}", pMP->mnId);
#endif
  // Create unique symbol
  gtsam::Symbol sym('l', pMP->mnId);
  // Create landmark position
  cv::Mat p_cv = pMP->GetWorldPos();
  gtsam::Point3 p_gtsam(p_cv.at<double>(0), p_cv.at<double>(1), p_cv.at<double>(2));

  if (values_.exists(sym.key()))
    values_.update(sym.key(), p_gtsam);
  else
    values_.insert(sym.key(), p_gtsam);
}

void GtsamTransformer::addMonoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                          ORB_SLAM2::MapPoint *pMP,
                                          Eigen::Matrix<double, 2, 1> &obs,
                                          const float inv_sigma_2) {
#ifdef DEBUG
  logger_->debug("addMonoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
#endif
  // Create both symbols
  gtsam::Symbol keyframe_sym('x', pKF->mnId);
  gtsam::Symbol landmark_sym('l', pMP->mnId);

  // Create landmark observation
  gtsam::Point2 obs_gtsam(obs(0), obs(1));

  // Create factor graph
  gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
      factor(obs_gtsam,
             gtsam::noiseModel::Diagonal::Variances(Eigen::Vector2d(1 / inv_sigma_2, 1 / inv_sigma_2)),
             keyframe_sym.key(),
             landmark_sym.key(),
             cam_params_mono_);
  session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = gtsam::serialize(factor);
}

void GtsamTransformer::addStereoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                            ORB_SLAM2::MapPoint *pMP,
                                            Eigen::Matrix<double, 3, 1> &obs,
                                            const float inv_sigma_2) {
#ifdef DEBUG
  logger_->debug("addStereoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
#endif
  // Create both symbols
  gtsam::Symbol keyframe_sym('x', pKF->mnId);
  gtsam::Symbol landmark_sym('l', pMP->mnId);

  // Create landmark observation
  gtsam::StereoPoint2 obs_gtsam(obs(0), obs(2), obs(1));

  // Create factor graph
  gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
      factor(obs_gtsam,
             gtsam::noiseModel::Diagonal::Variances(Eigen::Vector3d(1 / inv_sigma_2, 1 / inv_sigma_2, 1 / inv_sigma_2)),
             keyframe_sym.key(),
             landmark_sym.key(),
             cam_params_stereo_);
  session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = gtsam::serialize(factor);
}

void GtsamTransformer::setKeyFramePose(ORB_SLAM2::KeyFrame *pKF, g2o::SE3Quat pose) {
#ifdef DEBUG
  logger_->debug("setKeyFramePose - pKF->mnId: {}", pKF->mnId);
#endif
  // Create keyframe symbol
  gtsam::Symbol sym('x', pKF->mnId);

  // Create pose
  cv::Mat T_cv = Converter::toCvMat(pose);
  gtsam::Matrix4 T_gtsam;
  T_gtsam << T_cv.at<double>(0, 0), T_cv.at<double>(0, 1), T_cv.at<double>(0, 2), T_cv.at<double>(0, 3)
      , T_cv.at<double>(1, 0), T_cv.at<double>(1, 1), T_cv.at<double>(1, 2), T_cv.at<double>(1, 3)
      , T_cv.at<double>(2, 0), T_cv.at<double>(2, 1), T_cv.at<double>(2, 2), T_cv.at<double>(2, 3)
      , T_cv.at<double>(3, 0), T_cv.at<double>(3, 1), T_cv.at<double>(3, 2), T_cv.at<double>(3, 3);
  gtsam::Pose3 left_cam_pose(T_gtsam);
  gtsam::StereoCamera stereo_cam(left_cam_pose, cam_params_stereo_);

  values_.update(sym.key(), stereo_cam);
}

void GtsamTransformer::setLandmarkPose(ORB_SLAM2::MapPoint *pMP, g2o::Vector3d position) {
#ifdef DEBUG
  logger_->debug("setLandmarkPose - pMP->mnId: {}", pMP->mnId);
#endif
  // Create keyframe symbol
  gtsam::Symbol sym('l', pMP->mnId);

  // Create position
  gtsam::Point3 p_gtsam(position(0), position(1), position(2));

  values_.update(sym.key(), p_gtsam);
}

std::tuple<bool,
           boost::optional<bool>,
           boost::optional<std::vector<std::string>>,
           boost::optional<std::vector<std::pair<gtsam::Key, gtsam::Key>>>,
           boost::optional<std::set<gtsam::Key>>,
           boost::optional<std::set<gtsam::Key>>,
           boost::optional<gtsam::Values>> GtsamTransformer::checkForNewData() {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    if (new_optimized_data_) {
      logger_->info("checkForNewData - returning new optimized data");
      new_optimized_data_ = false;
      return std::make_tuple(true, is_full_BA_data_, add_factors_, del_factors_, add_states_, del_states_, values_);
    } else {
#ifdef DEBUG
      logger_->debug("checkForNewData - there is no new data.");
#endif
    }
  } else {
    logger_->error("checkForNewData - can't own mutex. returning false");
  }
  return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none);
}

void GtsamTransformer::handleKeyframe(ORB_SLAM2::KeyFrame *pKF, bool is_bad) {
  is_bad ? del_states_.insert(gtsam::Symbol('x', pKF->mnId)) : session_states_.insert(gtsam::Symbol('x', pKF->mnId));
}

void GtsamTransformer::handleLandmark(ORB_SLAM2::MapPoint *pMP, bool is_bad) {
  is_bad ? del_states_.insert(gtsam::Symbol('l', pMP->mnId)) : session_states_.insert(gtsam::Symbol('l', pMP->mnId));
}

void GtsamTransformer::removeFactor(ORB_SLAM2::KeyFrame *pKF, ORB_SLAM2::MapPoint *pMP) {
  gtsam::Symbol keyframe_sym('x', pKF->mnId);
  gtsam::Symbol landmark_sym('l', pMP->mnId);
  del_factors_.emplace_back(keyframe_sym.key(), landmark_sym.key());
#ifdef DEBUG
  logger_->debug("removeFactor - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
#endif
}

void GtsamTransformer::updateActiveSets() {
  // Handle added states
  if (active_states_.empty()) {
    add_states_ = session_states_;
  } else {
    std::set_difference(session_states_.begin(),
                        session_states_.end(),
                        active_states_.begin(),
                        active_states_.end(),
                        std::inserter(add_states_, add_states_.begin()));
  }

  // Handle deleted states
  std::set<gtsam::Key> new_del_states;
  std::set_intersection(active_states_.begin(),
                        active_states_.end(),
                        del_states_.begin(),
                        del_states_.end(),
                        std::inserter(new_del_states, new_del_states.begin()));
  del_states_ = new_del_states;

  // Update active set
  if (!del_states_.empty()) {
    for (const auto &it: del_states_) {
      active_states_.erase(it);
    }
  }
  active_states_.insert(add_states_.begin(), add_states_.end());

#ifdef DEBUG
  logger_->debug("updateActiveSets - add_states: {}", setToString(add_states_));
  logger_->debug("updateActiveSets - del_states: {}", setToString(del_states_));
#endif

  // Handle added factors
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> add_factors_map;
  if (active_factors_.empty()) {
    add_factors_map = session_factors_;
    exportValuesFromMap(add_factors_map, add_factors_);
  } else {
    std::set_difference(session_factors_.begin(),
                        session_factors_.end(),
                        active_factors_.begin(),
                        active_factors_.end(),
                        std::inserter(add_factors_map, add_factors_map.begin()),
                        session_factors_.value_comp());
    exportValuesFromMap(add_factors_map, add_factors_);
  }

  // Handle deleted factors
  std::vector<std::pair<gtsam::Key, gtsam::Key>> new_del_factors;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> active_factor_keys;
  exportKeysFromMap(active_factors_, active_factor_keys);
  std::set_intersection(active_factor_keys.begin(),
                        active_factor_keys.end(),
                        del_factors_.begin(),
                        del_factors_.end(),
                        std::inserter(new_del_factors, new_del_factors.begin()));
  del_factors_ = new_del_factors;

  // Update active factors
  if (!del_factors_.empty()) {
    for (const auto &it: del_factors_) {
      active_factors_.erase(it);
    }
  }
  active_factors_.insert(add_factors_map.begin(), add_factors_map.end());
}

void GtsamTransformer::start(bool is_full_BA) {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    logger_->info("start - new recovering session. is_full_BA: {}", is_full_BA);
    if (!new_optimized_data_) {
      add_states_.clear();
      del_states_.clear();
      session_states_.clear();

      add_factors_.clear();
      del_factors_.clear();
      session_factors_.clear();
    }

    new_optimized_data_ = false; // To make the data available only after finish processing
    is_full_BA_data_ = is_full_BA;
  } else {
    logger_->warn("start - can't own mutex. returns");
  }
}

void GtsamTransformer::finish() {
  std::unique_lock<std::mutex> *lock;
  do {
    lock = new std::unique_lock<std::mutex>(mutex_, std::try_to_lock);
  } while (!lock->owns_lock());
  logger_->info("finish - ending recovering session. new_optimized_data is now available");
  logger_->info("finish - active states set size: {}", active_states_.size());
  logger_->info("finish - active factors vector size: {}", active_factors_.size());
  updateActiveSets();
  new_optimized_data_ = true;
  delete lock;
}

void GtsamTransformer::exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> &map,
                                         std::vector<std::pair<gtsam::Key, gtsam::Key>> &output) {
  for (const auto &it: map) {
    output.push_back(it.first);
  }
}

void GtsamTransformer::exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::string> &map, std::vector<std::string> &output) {
  for (const auto &it: map) {
    output.push_back(it.second);
  }
}

std::string GtsamTransformer::setToString(const std::set<gtsam::Key> &set) const {
  std::stringstream ss;
  for (const auto &it: set)
    ss << it << " ";
  return ss.str();
}
}