//
// Created by Or Salmon on 15/08/18.
//

#include "AnplGtsamRecover.h"

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Optimizer.h"

#define DEBUG

namespace ORB_SLAM2 {
AnplGtsamRecover::AnplGtsamRecover() : PRIOR_SIGMA_2(0.01) {
  logger_ = spdlog::rotating_logger_st("AnplGtsamRecover",
                                       "AnplGtsamRecover",
                                       1048576 * 50,
                                       3);
  logger_->set_level(spdlog::level::debug);
  logger_->info("CTOR - AnplGtsamRecover instance created");
}

AnplGtsamRecover::~AnplGtsamRecover() {
  if ((graph_ != nullptr) && (values_ != nullptr)) {
    delete graph_;
    delete values_;
  }
}

void AnplGtsamRecover::addKeyFrame(ORB_SLAM2::KeyFrame *pKF, bool fixed_pose) {
#ifdef DEBUG
  logger_->debug("addKeyFrame - pKF->mnId: {}, fixed_pose: {}", pKF->mnId, fixed_pose);
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
  values_->insert(sym.key(), stereo_cam);

  if (fixed_pose) {
    gtsam::PriorFactor<gtsam::StereoCamera>
        prior(sym.key(),
              stereo_cam,
              gtsam::noiseModel::Diagonal::Variances(Eigen::VectorXd::Ones(6) * PRIOR_SIGMA_2));
    graph_->add(prior);
  }
}

void AnplGtsamRecover::addLandmark(ORB_SLAM2::MapPoint *pMP) {
#ifdef DEBUG
  logger_->debug("addLandmark - pMP->mnId: {}", pMP->mnId);
#endif
  // Create unique symbol
  gtsam::Symbol sym('l', pMP->mnId);
  // Create landmark position
  cv::Mat p_cv = pMP->GetWorldPos();
  gtsam::Point3 p_gtsam(p_cv.at<double>(0), p_cv.at<double>(1), p_cv.at<double>(2));

  values_->insert(sym.key(), p_gtsam);
}

void AnplGtsamRecover::addMonoMeasurement(ORB_SLAM2::KeyFrame *pKF,
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
  graph_->add(factor);
}

void AnplGtsamRecover::addStereoMeasurement(ORB_SLAM2::KeyFrame *pKF,
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
  graph_->add(factor);
}

void AnplGtsamRecover::setKeyFramePose(ORB_SLAM2::KeyFrame *pKF, g2o::SE3Quat pose) {
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

  values_->update(sym.key(), stereo_cam);
}

void AnplGtsamRecover::setLandmarkPose(ORB_SLAM2::MapPoint *pMP, g2o::Vector3d position) {
#ifdef DEBUG
  logger_->debug("setLandmarkPose - pMP->mnId: {}", pMP->mnId);
#endif
  // Create keyframe symbol
  gtsam::Symbol sym('l', pMP->mnId);

  // Create position
  gtsam::Point3 p_gtsam(position(0), position(1), position(2));

  values_->update(sym.key(), p_gtsam);
}

std::tuple<bool,
           boost::optional<bool>,
           boost::optional<std::string>,
           boost::optional<std::string>,
           boost::optional<std::set<unsigned long>>,
           boost::optional<std::set<unsigned long>>,
           boost::optional<std::set<unsigned long>>,
           boost::optional<std::set<unsigned long>>> AnplGtsamRecover::checkForNewData() {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    if (new_optimized_data_) {
      logger_->info("checkForNewData - returning new optimized data");
      std::string graph(std::move(serialized_graph_));
      std::string values(std::move(serialized_values_));
      return std::make_tuple(true, is_full_BA_data_, graph, values, add_keyframes_, del_keyframes_, add_landmarks_, del_landmarks_);
    } else {
#ifdef DEBUG
      logger_->debug("checkForNewData - there is no new data.");
#endif
    }
  } else {
    logger_->error("checkForNewData - can't own mutex. returning false");
  }
  return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none, boost::none);
}

void AnplGtsamRecover::handleKeyframe(ORB_SLAM2::KeyFrame *pKF, bool is_bad) {
  is_bad ? del_keyframes_.insert(pKF->mnId) : session_keyframes_.insert(pKF->mnId);
}

void AnplGtsamRecover::handleLandmark(ORB_SLAM2::MapPoint *pMP, bool is_bad) {
  is_bad ? del_landmarks_.insert(pMP->mnId) : session_landmarks_.insert(pMP->mnId);
}

void AnplGtsamRecover::updateActiveSets() {
  // Handle added items
  if (active_keyframes_.empty()) {
    add_keyframes_ = session_keyframes_;
  } else {
    std::set_difference(session_keyframes_.begin(),
                        session_keyframes_.end(),
                        active_keyframes_.begin(),
                        active_keyframes_.end(),
                        std::inserter(add_keyframes_, add_keyframes_.begin()));
  }
  if (active_landmarks_.empty()) {
    add_landmarks_ = session_landmarks_;
  } else {
    std::set_difference(session_landmarks_.begin(),
                        session_landmarks_.end(),
                        active_landmarks_.begin(),
                        active_landmarks_.end(),
                        std::inserter(add_landmarks_, add_landmarks_.begin()));
  }

  // Handle deleted items
  std::set<unsigned long> new_del_kf;
  std::set_intersection(active_keyframes_.begin(),
                        active_keyframes_.end(),
                        del_keyframes_.begin(),
                        del_keyframes_.end(),
                        std::inserter(new_del_kf, new_del_kf.begin()));
  del_keyframes_ = new_del_kf;
  std::set<unsigned long> new_del_mp;
  std::set_intersection(active_landmarks_.begin(),
                        active_landmarks_.end(),
                        del_landmarks_.begin(),
                        del_landmarks_.end(),
                        std::inserter(new_del_mp, new_del_mp.begin()));
  del_landmarks_ = new_del_mp;

  // Update active sets
  if (!del_keyframes_.empty()) {
    for (const auto &it: del_landmarks_) {
      active_landmarks_.erase(it);
    }
  }
  active_keyframes_.insert(add_keyframes_.begin(), add_keyframes_.end());
  if (!del_landmarks_.empty()) {
    for (const auto &it: del_landmarks_) {
      active_landmarks_.erase(it);
    }
  }
  active_landmarks_.insert(add_landmarks_.begin(), add_landmarks_.end());

  // Write to log (debug)
#ifdef DEBUG
  logger_->debug("updateActiveSets - add_kf: {}", setToString(add_keyframes_));
  logger_->debug("updateActiveSets - del_kf: {}", setToString(del_keyframes_));
  logger_->debug("updateActiveSets - add_mp: {}", setToString(add_landmarks_));
  logger_->debug("updateActiveSets - del_mp: {}", setToString(del_landmarks_));
#endif
}

void AnplGtsamRecover::start(bool is_full_BA) {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (lock.owns_lock()) {
    logger_->info("start - new recovering session. is_full_BA: {}", is_full_BA);
    if ((graph_ != nullptr) && (values_ != nullptr)) {
#ifdef DEBUG
      logger_->debug("start - clearing graph and values variables");
#endif
      delete graph_;
      delete values_;
    }
    graph_ = new gtsam::NonlinearFactorGraph;
    values_ = new gtsam::Values;

    add_keyframes_.clear();
    add_landmarks_.clear();
    del_keyframes_.clear();
    del_landmarks_.clear();

    new_optimized_data_ = false;
    is_full_BA_data_ = is_full_BA;
  } else {
    logger_->warn("start - can't own mutex. returns");
  }
}

void AnplGtsamRecover::finish() {
  std::unique_lock<std::mutex> *lock;
  do {
    lock = new std::unique_lock<std::mutex>(mutex_, std::try_to_lock);
  } while (!lock->owns_lock());
  logger_->info("finish - ending recovering session. new_optimized_data is now available");
  logger_->info("finish - active keyframes set size: {}", active_keyframes_.size());
  logger_->info("finish - active landmarks set size: {}", active_landmarks_.size());
  updateActiveSets();
//  serialized_graph_.assign(gtsam::serialize(*graph_));
//  serialized_values_.assign(gtsam::serialize(*values_));
  new_optimized_data_ = true;
#ifdef DEBUG
  logger_->debug("finish - graph:\n{}\nvalues:\n{}", serialized_graph_, serialized_values_);
  graph_->print();
#endif
  delete lock;
}

std::string AnplGtsamRecover::setToString(const std::set<unsigned long> &set) const {
  std::stringstream ss;
  for (const auto &it: set)
    ss << it << " ";
  return ss.str();
}
}