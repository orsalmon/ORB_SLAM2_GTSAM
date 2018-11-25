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
GtsamTransformer::GtsamTransformer() : is_first_time_(true) {
  logger_ = spdlog::rotating_logger_st("GtsamTransformer",
                                       "GtsamTransformer",
                                       1048576 * 50,
                                       3);
#ifdef DEBUG
  logger_->set_level(spdlog::level::debug);
#else
  logger_->set_level(spdlog::level::info);
#endif
  logger_->info("CTOR - GtsamTransformer instance created");
}

void GtsamTransformer::addKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
  logger_->debug("addKeyFrame - pKF->mnId: {}", pKF->mnId);
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
  Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> T_gtsam(T_cv.ptr<float>(), T_cv.rows, T_cv.cols);
  gtsam::Pose3 left_cam_pose(T_gtsam.cast<double>());
  gtsam::StereoCamera stereo_cam(left_cam_pose, cam_params_stereo_); // TODO: currently not used because adding factors issue

  if (values_.exists(sym.key()))
    values_.update(sym.key(), left_cam_pose);
  else
    values_.insert(sym.key(), left_cam_pose);
  add_values_.insert(sym.key(), left_cam_pose);

  if (pKF->mTimeStamp > std::get<1>(recent_kf_)) {
    recent_kf_ = std::make_tuple(gtsam::serialize(sym), pKF->mTimeStamp, gtsam::serialize(left_cam_pose));
  }
}

void GtsamTransformer::addLandmark(ORB_SLAM2::MapPoint *pMP) {
  logger_->debug("addLandmark - pMP->mnId: {}", pMP->mnId);
  // Create unique symbol
  gtsam::Symbol sym('l', pMP->mnId);
  // Create landmark position
  cv::Mat p_cv = pMP->GetWorldPos();
  gtsam::Point3 p_gtsam(p_cv.at<float>(0), p_cv.at<float>(1), p_cv.at<float>(2));

  if (values_.exists(sym.key()))
    values_.update(sym.key(), p_gtsam);
  else
    values_.insert(sym.key(), p_gtsam);
  add_values_.insert(sym.key(), p_gtsam);
}

void GtsamTransformer::addMonoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                          ORB_SLAM2::MapPoint *pMP,
                                          Eigen::Matrix<double, 2, 1> &obs,
                                          const float inv_sigma_2) {
  logger_->debug("addMonoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
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
  session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = std::make_pair(gtsam::serialize(factor), false);
}

void GtsamTransformer::addStereoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                            ORB_SLAM2::MapPoint *pMP,
                                            Eigen::Matrix<double, 3, 1> &obs,
                                            const float inv_sigma_2) {
  logger_->debug("addStereoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
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
  session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = std::make_pair(gtsam::serialize(factor), true);
}

void GtsamTransformer::setKeyFramePose(ORB_SLAM2::KeyFrame *pKF, g2o::SE3Quat pose) {
  logger_->debug("setKeyFramePose - pKF->mnId: {}", pKF->mnId);
  // Create keyframe symbol
  gtsam::Symbol sym('x', pKF->mnId);

  // Create pose
  cv::Mat T_cv = Converter::toCvMat(pose);
  Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> T_gtsam(T_cv.ptr<float>(), T_cv.rows, T_cv.cols);
  gtsam::Pose3 left_cam_pose(T_gtsam.cast<double>());
  gtsam::StereoCamera stereo_cam(left_cam_pose, cam_params_stereo_);

  values_.update(sym.key(), left_cam_pose);
  add_values_.update(sym.key(), left_cam_pose);

  if (gtsam::serialize(sym) == std::get<0>(recent_kf_))
    std::get<2>(recent_kf_) = gtsam::serialize(left_cam_pose);
}

void GtsamTransformer::setLandmarkPose(ORB_SLAM2::MapPoint *pMP, g2o::Vector3d position) {
  logger_->debug("setLandmarkPose - pMP->mnId: {}", pMP->mnId);
  // Create keyframe symbol
  gtsam::Symbol sym('l', pMP->mnId);

  // Create position
  gtsam::Point3 p_gtsam(position(0), position(1), position(2));

  values_.update(sym.key(), p_gtsam);
  add_values_.update(sym.key(), p_gtsam);
}

std::tuple<bool,
           boost::optional<bool>,
           boost::optional<std::string>,
           boost::optional<std::vector<std::pair<gtsam::Key, gtsam::Key>>>,
           boost::optional<std::set<gtsam::Key>>,
           boost::optional<std::set<gtsam::Key>>,
           boost::optional<std::string>,
           boost::optional<std::tuple<std::string, double, std::string>>> GtsamTransformer::checkForNewData() {
  if (ready_data_queue_.empty()) {
    logger_->debug("checkForNewData - there is no new data.");
    return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none);
  }
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    logger_->info("checkForNewData - returning new optimized data");
    auto data = ready_data_queue_.front();
    ready_data_queue_.pop();
    return data;
  } else {
    logger_->error("checkForNewData - can't own mutex. returning false");
    return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none);
  }
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
  logger_->debug("removeFactor - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
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

  logger_->debug("updateActiveSets - add_states: {}", setToString(add_states_));
  logger_->debug("updateActiveSets - del_states: {}", setToString(del_states_));

  // Handle added factors
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, bool>> add_factors_map;
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
    add_states_.clear();
    del_states_.clear();
    session_states_.clear();

    add_factors_.clear();
    del_factors_.clear();
    session_factors_.clear();

    add_values_.clear();
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
  ready_data_queue_.emplace(true,
                            is_full_BA_data_,
                            gtsam::serialize(createFactorGraph(add_factors_)),
                            del_factors_,
                            add_states_,
                            del_states_,
                            gtsam::serialize(add_values_),
                            recent_kf_);
  delete lock;
}

void GtsamTransformer::exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, bool>> &map,
                                         std::vector<std::pair<gtsam::Key, gtsam::Key>> &output) {
  for (const auto &it: map) {
    output.push_back(it.first);
  }
}

void GtsamTransformer::exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, bool>> &map,
                                           std::vector<std::pair<std::string, bool>> &output) {
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

gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(std::vector<std::pair<std::string, bool>> ser_factors_vec) {
  gtsam::NonlinearFactorGraph graph;

  // Adding fixed factor for x0
  if (is_first_time_) {
    is_first_time_ = false;
    gtsam::Pose3 x0_pose;
    gtsam::Symbol x0_sym('x',0);
    if (add_values_.exists(x0_sym.key()))
      x0_pose = add_values_.at<gtsam::Pose3>(x0_sym.key());
    graph.push_back(gtsam::NonlinearEquality<gtsam::Pose3>(gtsam::Symbol('x',0),x0_pose));
  }

  for (const auto &it: ser_factors_vec) {
    // Stereo factor
    if (it.second) {
      gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> factor;
      gtsam::deserialize(it.first, factor);
      graph.push_back(factor);
    }
      // Mono factor
    else {
      gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> factor;
      gtsam::deserialize(it.first, factor);
      graph.push_back(factor);
    }
  }
  return graph;
}
}