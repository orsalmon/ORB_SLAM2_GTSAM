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
  between_factors_prior_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e2, 1e2, 1e2, 1, 1, 1).finished());
}

void GtsamTransformer::addMonoMeasurement(ORB_SLAM2::KeyFrame *pKF,
                                          ORB_SLAM2::MapPoint *pMP,
                                          Eigen::Matrix<double, 2, 1> &obs,
                                          const float inv_sigma_2) {
  logger_->debug("addMonoMeasurement - pKF->mnId: {}, pMP->mnId: {}", pKF->mnId, pMP->mnId);
  if (!is_cam_params_initialized_) {
    std::cout << "addMonoMeasurement - camera params has not been initialized!" << std::endl;
    exit(-2);
  }
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
  if (!is_cam_params_initialized_) {
    std::cout << "addStereoMeasurement - camera params has not been initialized!" << std::endl;
    exit(-2);
  }
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

std::tuple<bool,
           boost::optional<bool>,
           boost::optional<std::string>,
           boost::optional<std::vector<size_t>>,
           boost::optional<const gtsam::KeyVector>,
           boost::optional<const gtsam::KeyVector>,
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

bool GtsamTransformer::start() {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    logger_->info("start - new recovering session.");
    add_states_.clear();
    del_states_.clear();
    session_values_.clear();

    add_factors_.clear();
    del_factors_.clear();
    session_factors_.clear();
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
  logger_->info("finish - active states set size: {}", session_values_.size());
  logger_->info("finish - active factors vector size: {}", session_factors_.size());
  if (update_type_ == INCREMENTAL) {
    // Incremental update
    auto incremental_factor_graph = createFactorGraph(add_factors_, true);
    ready_data_queue_.emplace(true,
                              true,
                              gtsam::serialize(incremental_factor_graph),
                              createDeletedFactorsIndicesVec(del_factors_),
                              add_states_,
                              del_states_,
                              gtsam::serialize(session_values_),
                              recent_kf_);
  } else if (update_type_ == BATCH) {
    // Batch update
    auto active_factor_graph = createFactorGraph(session_factors_, false);
    ready_data_queue_.emplace(true,
                              false,
                              gtsam::serialize(active_factor_graph),
                              createDeletedFactorsIndicesVec(del_factors_),
                              add_states_,
                              del_states_,
                              gtsam::serialize(session_values_),
                              recent_kf_);
  }
  logger_->info("finish - ready_data_queue.size: {}", ready_data_queue_.size());

  std::cout << "finish - session_factors.size: " << session_factors_.size() << " last_session_factors.size: " << last_session_factors_.size()
            << " add_factors.size: " << add_factors_.size()
            << " del_factors.size: " << del_factors_.size() << " add_states.size: " << add_states_.size() << " del_states.size: "
            << del_states_.size() << " values.size: " << session_values_.size() << " last_values.size: " << last_session_values_.size() << std::endl;

  last_session_values_ = session_values_;
  last_session_factors_ = session_factors_;

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

gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(std::vector<std::pair<std::string, FactorType>> ser_factors_vec,
                                                                bool is_incremental) {
  // In use only in batch mode (not incremental)
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> new_active_factors;

  if (!is_incremental) {
    current_index_ = 0;
    factor_indecies_dict_.clear();
  }
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
        if (!is_incremental && (del_factors_.size() > 0 || del_states_.size() > 0)) {
          gtsam::Symbol first_sym(between_factor.keys().at(0));
          gtsam::Symbol second_sym(between_factor.keys().at(1));
          if ((std::find(del_factors_.begin(), del_factors_.end(), std::make_pair(first_sym.key(), second_sym.key())) != del_factors_.end())
              || (std::find(del_states_.begin(), del_states_.end(), first_sym.key()) != del_states_.end())
              || (std::find(del_states_.begin(), del_states_.end(), second_sym.key()) != del_states_.end())) {
            break;
          } else {
            new_active_factors[std::make_pair(first_sym.key(), second_sym.key())] = it;
          }
        }
        graph.push_back(between_factor);
        factor_indecies_dict_[std::make_pair(between_factor.keys()[0], between_factor.keys()[1])] = current_index_++;
        break;
      }
      case FactorType::MONO: {
        gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> mono_factor;
        gtsam::deserialize(it.first, mono_factor);
        if (!is_incremental && (del_factors_.size() > 0 || del_states_.size() > 0)) {
          gtsam::Symbol first_sym(mono_factor.keys().at(0));
          gtsam::Symbol second_sym(mono_factor.keys().at(1));
          if ((std::find(del_factors_.begin(), del_factors_.end(), std::make_pair(first_sym.key(), second_sym.key())) != del_factors_.end())
              || (std::find(del_states_.begin(), del_states_.end(), first_sym.key()) != del_states_.end())
              || (std::find(del_states_.begin(), del_states_.end(), second_sym.key()) != del_states_.end())) {
            break;
          } else {
            new_active_factors[std::make_pair(first_sym.key(), second_sym.key())] = it;
          }
        }
        graph.push_back(mono_factor);
        factor_indecies_dict_[std::make_pair(mono_factor.keys()[0], mono_factor.keys()[1])] = current_index_++;
        break;
      }
      case FactorType::STEREO: {
        gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> stereo_factor;
        gtsam::deserialize(it.first, stereo_factor);
        if (!is_incremental && (del_factors_.size() > 0 || del_states_.size() > 0)) {
          gtsam::Symbol first_sym(stereo_factor.keys().at(0));
          gtsam::Symbol second_sym(stereo_factor.keys().at(1));
          if ((std::find(del_factors_.begin(), del_factors_.end(), std::make_pair(first_sym.key(), second_sym.key())) != del_factors_.end())
              || (std::find(del_states_.begin(), del_states_.end(), first_sym.key()) != del_states_.end())
              || (std::find(del_states_.begin(), del_states_.end(), second_sym.key()) != del_states_.end())) {
            break;
          } else {
            new_active_factors[std::make_pair(first_sym.key(), second_sym.key())] = it;
          }
        }
        graph.push_back(stereo_factor);
        factor_indecies_dict_[std::make_pair(stereo_factor.keys()[0], stereo_factor.keys()[1])] = current_index_++;
        break;
      }
    }
  }
  std::cout << "createFactorGraph - size: " << graph.size() << std::endl;
  if (!is_incremental) {
    session_factors_ = new_active_factors;

    for (const auto &it: del_states_) {
      if (session_values_.find(it) != session_values_.end()) {
        session_values_.erase(it);
      }
    }
  }
  return graph;
}

gtsam::NonlinearFactorGraph GtsamTransformer::createFactorGraph(map<pair<gtsam::Key, gtsam::Key>,
                                                                    pair<string, ORB_SLAM2::GtsamTransformer::FactorType>> ser_factors_map,
                                                                bool is_incremental) {
  std::vector<std::pair<std::string, FactorType>> ser_factors_vec;
  for (const auto &it: ser_factors_map)
    ser_factors_vec.push_back(it.second);

  return createFactorGraph(ser_factors_vec, is_incremental);
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

void GtsamTransformer::transformGraphToGtsam(const vector<ORB_SLAM2::KeyFrame *> &vpKFs, const vector<ORB_SLAM2::MapPoint *> &vpMP) {
  if (!start())
    return;
  for (const auto &pKF: vpKFs) {
    if (pKF->isBad())
      continue;
    updateKeyFrame(pKF, true);
  }
  for (const auto &pMP: vpMP) {
    if (pMP->isBad())
      continue;
    updateLandmark(pMP);
    const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();
    updateObservations(pMP, observations);
  }
  calculateDiffrencesBetweenValueSets();
  calculateDiffrencesBetweenFactorSets();
  finish();
}

void GtsamTransformer::updateKeyFrame(ORB_SLAM2::KeyFrame *pKF, bool add_between_factor) {
  // Create keyframe symbol
  gtsam::Symbol sym('x', pKF->mnId);

  // Create camera parameters
  if (!is_cam_params_initialized_) {
    cam_params_stereo_.reset(new gtsam::Cal3_S2Stereo(pKF->fx, pKF->fy, 0.0, pKF->cx, pKF->cy, pKF->mb));
    cam_params_mono_.reset(new gtsam::Cal3_S2(cam_params_stereo_->calibration()));
    is_cam_params_initialized_ = true;
  }

  // Create pose
  cv::Mat T_cv = pKF->GetPose();
  Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>> T_gtsam(T_cv.ptr<float>(), T_cv.rows, T_cv.cols);
  gtsam::Pose3 left_cam_pose(T_gtsam.cast<double>());
  gtsam::StereoCamera stereo_cam(left_cam_pose, cam_params_stereo_);

  session_values_.insert(sym.key(), stereo_cam.pose());

  // Adding prior factor for x0
  if (pKF->mnId == 0) {
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    gtsam::PriorFactor<gtsam::Pose3> prior_factor(gtsam::Symbol('x', 0), stereo_cam.pose(), prior_noise);
    session_factors_[std::make_pair(sym.key(), sym.key())] = std::make_pair(gtsam::serialize(prior_factor), FactorType::PRIOR);
  }

  // Adding between factor
  if (add_between_factor) {
    if (pKF->mnId != 0) {
      gtsam::Symbol sym_before('x', pKF->mnId - 1);
      if (session_values_.exists(sym_before.key())) {
        gtsam::Pose3 relative_pose = stereo_cam.pose().between(session_values_.at<gtsam::Pose3>(sym_before.key())).between(gtsam::Pose3());
        gtsam::BetweenFactor<gtsam::Pose3> between_factor(sym_before, sym, relative_pose, between_factors_prior_);
        session_factors_[std::make_pair(sym_before.key(), sym.key())] = std::make_pair(gtsam::serialize(between_factor), FactorType::BETWEEN);
      }
    }
  }

  // Update most recent keyframe
  if ((pKF->mTimeStamp > std::get<1>(recent_kf_)) || (pKF->mnId == 0)) {
    recent_kf_ = std::make_tuple(gtsam::serialize(sym), pKF->mTimeStamp, gtsam::serialize(stereo_cam.pose()));
  }
}

void GtsamTransformer::updateLandmark(ORB_SLAM2::MapPoint *pMP) {
  // Create landmark symbol
  gtsam::Symbol sym('l', pMP->mnId);

  // Create landmark position
  cv::Mat p_cv = pMP->GetWorldPos();
  gtsam::Point3 p_gtsam(p_cv.at<float>(0), p_cv.at<float>(1), p_cv.at<float>(2));

  session_values_.insert(sym.key(), p_gtsam);
}

void GtsamTransformer::updateObservations(MapPoint *pMP, const map<ORB_SLAM2::KeyFrame *, size_t> &observations) {
  for (const auto &mit: observations) {
    KeyFrame *pKFi = mit.first;
    if (pKFi->isBad())
      continue;
    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit.second];
    // Monocular observation
    if (pKFi->mvuRight[mit.second] < 0) {
      Eigen::Matrix<double, 2, 1> obs;
      obs << kpUn.pt.x, kpUn.pt.y;
      const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
      addMonoMeasurement(pKFi, pMP, obs, invSigma2);
    } else // Stereo observation
    {
      Eigen::Matrix<double, 3, 1> obs;
      const float kp_ur = pKFi->mvuRight[mit.second];
      obs << kpUn.pt.x, kpUn.pt.y, kp_ur;
      const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
      addStereoMeasurement(pKFi, pMP, obs, invSigma2);
    }
  }
}

void GtsamTransformer::calculateDiffrencesBetweenValueSets() {
  // Handle added states
  if (last_session_values_.empty()) {
    add_states_ = session_values_.keys();
  } else {
    add_states_ = getDifferenceKeyList(session_values_.keys(), last_session_values_.keys());
  }

  // Handle deleted states
  del_states_ = getDifferenceKeyList(last_session_values_.keys(), session_values_.keys());
}

void GtsamTransformer::calculateDiffrencesBetweenFactorSets() {
  // Handle added factors
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>> add_factors_map;
  if (last_session_factors_.empty()) {
    add_factors_map = session_factors_;
    exportValuesFromMap(add_factors_map, add_factors_);
  } else {
    add_factors_map = getDifferenceSet(session_factors_, last_session_factors_);
    exportValuesFromMap(add_factors_map, add_factors_);
  }

  // Handle deleted factors
  std::map<std::pair<gtsam::Key, gtsam::Key>, std::pair<std::string, FactorType>>
      del_factors_map = getDifferenceSet(last_session_factors_, session_factors_);
  exportKeysFromMap(del_factors_map, del_factors_);
}

gtsam::KeyVector GtsamTransformer::getDifferenceKeyList(const gtsam::KeyVector &vec_A, const gtsam::KeyVector &vec_B) {
  gtsam::KeyVector diff_vec;
  for (const auto &it_A: vec_A) {
    if (std::find(vec_B.begin(), vec_B.end(), it_A) == vec_B.end()) {
      diff_vec.push_back(it_A);
    }
  }
  return diff_vec;
}

void GtsamTransformer::setUpdateType(const ORB_SLAM2::GtsamTransformer::UpdateType update_type) {
  update_type_ = update_type;
  if (update_type_ == BATCH) {
    logger_->info("setUpdateType - Batch");
  } else if (update_type_ == INCREMENTAL) {
    logger_->info("setUpdateType - Incremental");
  }
}
}