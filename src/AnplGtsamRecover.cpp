//
// Created by Or Salmon on 15/08/18.
//

#include "AnplGtsamRecover.h"

#include "KeyFrame.h"
#include "MapPoint.h"
#include "Optimizer.h"

namespace ORB_SLAM2 {
AnplGtsamRecover::AnplGtsamRecover() {}

AnplGtsamRecover::~AnplGtsamRecover() {
  delete graph_;
  delete values_;
}

void AnplGtsamRecover::addKeyFrame(ORB_SLAM2::KeyFrame *pKF, bool initiate_with_pose) {
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

  if (initiate_with_pose) {
    gtsam::PriorFactor<gtsam::StereoCamera> prior(sym.key(), stereo_cam, cam_pose_prior_noise_model_);
    graph_->add(prior);
  }
}

void AnplGtsamRecover::addLandmark(ORB_SLAM2::MapPoint *pMP, bool initiate_with_pose) {
  // Create unique symbol
  gtsam::Symbol sym('l', pMP->mnId);
  // Create landmark position
  cv::Mat p_cv = pMP->GetWorldPos();
  gtsam::Point3 p_gtsam(p_cv.at<double>(0), p_cv.at<double>(1), p_cv.at<double>(2));

  values_->insert(sym.key(), p_gtsam);

  if (initiate_with_pose) {
    gtsam::PriorFactor<gtsam::Point3> prior(sym.key(), p_gtsam, landmark_position_prior_noise_model_);
    graph_->add(prior);
  }
}

void AnplGtsamRecover::addMonoMeasurement(ORB_SLAM2::KeyFrame *pKF, ORB_SLAM2::MapPoint *pMP, Eigen::Matrix<double, 2, 1> &obs) {
  // Create both symbols
  gtsam::Symbol keyframe_sym('x', pKF->mnId);
  gtsam::Symbol landmark_sym('l', pMP->mnId);

  // Create landmark observation
  gtsam::Point2 obs_gtsam(obs(0), obs(1));

  // Create factor graph
  gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>
      factor(obs_gtsam, mono_observation_noise_model_, keyframe_sym.key(), landmark_sym.key(), cam_params_mono_);
  graph_->add(factor);
}

void AnplGtsamRecover::addStereoMeasurement(ORB_SLAM2::KeyFrame *pKF, ORB_SLAM2::MapPoint *pMP, Eigen::Matrix<double, 3, 1> &obs) {
  // Create both symbols
  gtsam::Symbol keyframe_sym('x', pKF->mnId);
  gtsam::Symbol landmark_sym('l', pMP->mnId);

  // Create landmark observation
  gtsam::StereoPoint2 obs_gtsam(obs(0), obs(2), obs(1));

  // Create factor graph
  gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3>
      factor(obs_gtsam, stereo_observation_noise_model_, keyframe_sym.key(), landmark_sym.key(), cam_params_stereo_);
  graph_->add(factor);
}

void AnplGtsamRecover::setKeyFramePose(ORB_SLAM2::KeyFrame *pKF, g2o::SE3Quat pose) {
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

  values_->update(sym.key(), left_cam_pose);
}

void AnplGtsamRecover::setLandmarkPose(ORB_SLAM2::MapPoint *pMP, g2o::Vector3d position) {
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
           boost::optional<std::vector<gtsam::Key>>> AnplGtsamRecover::checkForNewData() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (new_optimized_data_) {
    std::string graph(std::move(serialized_graph_));
    std::string values(std::move(serialized_values_));
    return std::make_tuple(true, is_full_BA_data_, graph, values, del_keyframes_);
  }
  return std::make_tuple(false, boost::none, boost::none, boost::none, boost::none);
}

void AnplGtsamRecover::start(bool is_full_BA) {
  std::lock_guard<std::mutex> lock(mutex_);
  delete graph_;
  delete values_;
  graph_ = new gtsam::NonlinearFactorGraph;
  values_ = new gtsam::Values;

  del_keyframes_.clear();

  new_optimized_data_ = false;
  is_full_BA_data_ = is_full_BA;
}

void AnplGtsamRecover::finish() {
  std::lock_guard<std::mutex> lock(mutex_);
  serialized_graph_.assign(gtsam::serialize(*graph_));
  serialized_values_.assign(gtsam::serialize(*values_));
  new_optimized_data_ = true;
}
}