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
                                       "GtsamTransformer.log",
                                       1048576 * 50,
                                       3);
#ifdef DEBUG
  logger_->set_level(spdlog::level::debug);
#else
  logger_->set_level(spdlog::level::info);
#endif
  logger_->info("CTOR - GtsamTransformer instance created");
  between_factors_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e9, 1e9, 1e9, 1e9, 1e9, 1e9));
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
  else {
    values_.insert(sym.key(), left_cam_pose);
    add_values_.insert(sym.key(), left_cam_pose);
  }

  if (pKF->mTimeStamp > std::get<1>(recent_kf_)) {
    recent_kf_ = std::make_tuple(gtsam::serialize(sym), pKF->mTimeStamp, gtsam::serialize(left_cam_pose));
  }

  if (pKF->mnId == 0) {
    // Adding prior factor for x0
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6));
    gtsam::PriorFactor<gtsam::Pose3> prior_factor(gtsam::Symbol('x', 0), left_cam_pose, prior_noise);
    session_factors_[std::make_pair(sym.key(), sym.key())] = std::make_pair(gtsam::serialize(prior_factor), FactorType::PRIOR);
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
  else {
    values_.insert(sym.key(), p_gtsam);
    add_values_.insert(sym.key(), p_gtsam);
  }
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
  session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = std::make_pair(gtsam::serialize(factor), FactorType::MONO);
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
  session_factors_[std::make_pair(keyframe_sym.key(), landmark_sym.key())] = std::make_pair(gtsam::serialize(factor), FactorType::STEREO);
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
  if (add_values_.exists(sym.key()))
    add_values_.update(sym.key(), left_cam_pose);

  // Creating between factors
  if (pKF->mnId != 0) {
    gtsam::Symbol sym_before('x', pKF->mnId - 1);
    if (values_.exists(sym_before.key())) {
      gtsam::Pose3 relative_pose = left_cam_pose.between(values_.at<gtsam::Pose3>(sym_before.key())).between(gtsam::Pose3());
      gtsam::BetweenFactor<gtsam::Pose3> between_factor(sym_before, sym, relative_pose, between_factors_prior_);
      session_factors_[std::make_pair(sym_before.key(), sym.key())] = std::make_pair(gtsam::serialize(between_factor), FactorType::BETWEEN);
    }
  }

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
  if (add_values_.exists(sym.key()))
    add_values_.update(sym.key(), p_gtsam);
}

std::tuple<bool,
           boost::optional<bool>,
           boost::optional<std::string>,
           boost::optional<std::vector<size_t>>,
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
    logger_->info("checkForNewData - returning new optimized data. ready_data_queue.size: {}", ready_data_queue_.size());
    auto data = ready_data_queue_.front();
    ready_data_queue_.pop();
    std::cout << "checkForNewData - returns " << (std::get<1>(data) ? "Incremental" : "Batch") << " update" << std::endl;
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
  std::cout << "removeFactor - " << keyframe_sym.chr() << keyframe_sym.index() << "-" << landmark_sym.chr() << landmark_sym.index() << std::endl;
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
  active_states_.insert(add_states_.begin(), add_states_.end());

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

  logger_->debug("updateActiveSets - add_states: {}", setToString(add_states_));
  logger_->debug("updateActiveSets - del_states: {}", setToString(del_states_));

  // Handle added factors
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> add_factors_map;
  if (active_factors_.empty()) {
    add_factors_map = session_factors_;
    exportValuesFromMap(add_factors_map, add_factors_);
  } else {
    add_factors_map = getDifferenceSet(session_factors_, active_factors_);
    exportValuesFromMap(add_factors_map, add_factors_);
  }
  active_factors_.insert(add_factors_map.begin(), add_factors_map.end());

  // Handle deleted factors
  del_factors_ = getIntersectionVec(active_factors_, del_factors_);

  // Update active factors
  if (!del_factors_.empty()) {
    for (const auto &it: del_factors_) {
      gtsam::Symbol key1(it.first);
      gtsam::Symbol key2(it.second);
      std::cout << "updateActiveSets - " << key1.chr() << key1.index() << "-" << key2.chr() << key2.index() << std::endl;

      active_factors_.erase(it);
    }
  }
}

bool GtsamTransformer::start() {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    logger_->info("start - new recovering session.");
    add_states_.clear();
    del_states_.clear();
    session_states_.clear();

    add_factors_.clear();
    del_factors_.clear();
    session_factors_.clear();

    add_values_.clear();
    return true;
  } else {
    logger_->warn("start - can't own mutex. returns");
  }
  return false;
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
  if (del_states_.size() == 0) {
    // Incremental update
    ready_data_queue_.emplace(true,
                              true,
                              gtsam::serialize(createFactorGraph(add_factors_)),
                              createDeletedFactorsIndicesVec(del_factors_),
                              add_states_,
                              del_states_,
                              gtsam::serialize(add_values_),
                              recent_kf_);
  } else {
    // Batch update
    ready_data_queue_.emplace(true,
                              false,
                              gtsam::serialize(createFactorGraph(active_factors_)),
                              createDeletedFactorsIndicesVec(del_factors_),
                              add_states_,
                              del_states_,
                              gtsam::serialize(values_),
                              recent_kf_);
  }
  logger_->info("finish - ready_data_queue.size: {}", ready_data_queue_.size());

  std::cout << "finish - active_factors.size: " << active_factors_.size() << " session_factors.size: " << session_factors_.size()
            << " add_factors.size: " << add_factors_.size()
            << " del_factors.size: " << del_factors_.size() << " add_states.size: " << add_states_.size() << " del_states.size: "
            << del_states_.size() << " add_values.size: " << add_values_.size() << std::endl;
  delete lock;
}

void GtsamTransformer::exportKeysFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                                         std::vector<std::pair<gtsam::Key, gtsam::Key>> &output) {
  for (const auto &it: map) {
    output.push_back(it.first);
  }
}

void GtsamTransformer::exportValuesFromMap(std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> &map,
                                           std::vector<std::pair<std::string, FactorType>> &output) {
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

gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(std::vector<std::pair<std::string, FactorType>> ser_factors_vec) {
  gtsam::NonlinearFactorGraph graph;
  for (const auto &it: ser_factors_vec) {
    switch (it.second) {
      case FactorType::PRIOR: {
        gtsam::PriorFactor<gtsam::Pose3> prior_factor;
        gtsam::deserialize(it.first, prior_factor);
        graph.push_back(prior_factor);
        factor_indecies_dict_[std::make_pair(prior_factor.keys()[0], prior_factor.keys()[1])] = current_index_++;
        break;
      }
      case FactorType::BETWEEN: {
        gtsam::BetweenFactor<gtsam::Pose3> between_factor;
        gtsam::deserialize(it.first, between_factor);
        graph.push_back(between_factor);
        factor_indecies_dict_[std::make_pair(between_factor.keys()[0], between_factor.keys()[1])] = current_index_++;
        break;
      }
      case FactorType::MONO: {
        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> mono_factor;
        gtsam::deserialize(it.first, mono_factor);
        graph.push_back(mono_factor);
        factor_indecies_dict_[std::make_pair(mono_factor.keys()[0], mono_factor.keys()[1])] = current_index_++;
        break;
      }
      case FactorType::STEREO: {
        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> stereo_factor;
        gtsam::deserialize(it.first, stereo_factor);
        graph.push_back(stereo_factor);
        factor_indecies_dict_[std::make_pair(stereo_factor.keys()[0], stereo_factor.keys()[1])] = current_index_++;
        break;
      }
    }
  }
  std::cout << "createFactorGraph - size: " << graph.size() << std::endl;
  return graph;
}

gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(map<pair<gtsam::Key, gtsam::Key>,
                                                                    pair<string, ORB_SLAM2::GtsamTransformer::FactorType>> ser_factors_map) {
  std::vector<std::pair<std::string, FactorType>> ser_factors_vec;
  for (const auto &it: ser_factors_map)
    ser_factors_vec.push_back(it.second);

  return createFactorGraph(ser_factors_vec);
}

std::vector<size_t> GtsamTransformer::createDeletedFactorsIndicesVec(std::vector<std::pair<gtsam::Key, gtsam::Key>> &del_factors) {
  std::vector<size_t> deleted_factors_indecies;
  for (const auto &it: del_factors) {
    auto dict_it = factor_indecies_dict_.find(it);
    if (dict_it != factor_indecies_dict_.end()) {
      deleted_factors_indecies.push_back(dict_it->second);

      gtsam::Symbol key1(it.first);
      gtsam::Symbol key2(it.second);
      std::cout << "createDeletedFactorsIndicesVec - " << key1.chr() << key1.index() << "-" << key2.chr() << key2.index() << " index: "
                << dict_it->second << std::endl;
    }
  }
  return deleted_factors_indecies;
}

map<pair<gtsam::Key, gtsam::Key>, pair<string, GtsamTransformer::FactorType>> GtsamTransformer::getDifferenceSet(map<pair<gtsam::Key, gtsam::Key>,
                                                                                                                     pair<string,
                                                                                                                          ORB_SLAM2::GtsamTransformer::FactorType>> &set_A,
                                                                                                                 map<pair<gtsam::Key, gtsam::Key>,
                                                                                                                     pair<string,
                                                                                                                          ORB_SLAM2::GtsamTransformer::FactorType>> &set_B) {
  map<pair<gtsam::Key, gtsam::Key>, pair<string, GtsamTransformer::FactorType>> diff_set;
  for (const auto &it_A: set_A) {
    if (set_B.find(it_A.first) == set_B.end()) {
      diff_set.insert(it_A);
    }
  }
  return diff_set;
}

std::vector<std::pair<gtsam::Key, gtsam::Key>> GtsamTransformer::getIntersectionVec(std::map<std::pair<gtsam::Key, gtsam::Key>,
                                                                                             std::pair<std::string, FactorType>> &set_A,
                                                                                    std::vector<std::pair<gtsam::Key, gtsam::Key>> &vec_B) {
  std::vector<std::pair<gtsam::Key, gtsam::Key>> inter_vec;
  for (const auto &it_B: vec_B) {
    if (set_A.find(it_B) != set_A.end()) {
      inter_vec.push_back(it_B);
    }
  }
  return inter_vec;
}
}