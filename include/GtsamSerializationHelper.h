#ifndef GTSAM_SERIALIZATION_HELPER_H
#define GTSAM_SERIALIZATION_HELPER_H

/* ************************************************************************ */
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianISAM.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/base/LieMatrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/StereoCamera.h>

#include <gtsam/base/serializationTestHelpers.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/serialization/version.hpp>

// Creating as many permutations of factors as possible
typedef gtsam::PriorFactor<gtsam::LieVector>         PriorFactorLieVector;
typedef gtsam::PriorFactor<gtsam::LieMatrix>         PriorFactorLieMatrix;
typedef gtsam::PriorFactor<gtsam::Point2>            PriorFactorPoint2;
typedef gtsam::PriorFactor<gtsam::StereoPoint2>      PriorFactorStereoPoint2;
typedef gtsam::PriorFactor<gtsam::Point3>            PriorFactorPoint3;
typedef gtsam::PriorFactor<gtsam::Rot2>              PriorFactorRot2;
typedef gtsam::PriorFactor<gtsam::Rot3>              PriorFactorRot3;
typedef gtsam::PriorFactor<gtsam::Pose2>             PriorFactorPose2;
typedef gtsam::PriorFactor<gtsam::Pose3>             PriorFactorPose3;
typedef gtsam::PriorFactor<gtsam::Cal3_S2>           PriorFactorCal3_S2;
typedef gtsam::PriorFactor<gtsam::Cal3DS2>           PriorFactorCal3DS2;
typedef gtsam::PriorFactor<gtsam::CalibratedCamera>  PriorFactorCalibratedCamera;
typedef gtsam::PriorFactor<gtsam::SimpleCamera>      PriorFactorSimpleCamera;
typedef gtsam::PriorFactor<gtsam::StereoCamera>      PriorFactorStereoCamera;

typedef gtsam::BetweenFactor<gtsam::LieVector>       BetweenFactorLieVector;
typedef gtsam::BetweenFactor<gtsam::LieMatrix>       BetweenFactorLieMatrix;
typedef gtsam::BetweenFactor<gtsam::Point2>          BetweenFactorPoint2;
typedef gtsam::BetweenFactor<gtsam::Point3>          BetweenFactorPoint3;
typedef gtsam::BetweenFactor<gtsam::Rot2>            BetweenFactorRot2;
typedef gtsam::BetweenFactor<gtsam::Rot3>            BetweenFactorRot3;
typedef gtsam::BetweenFactor<gtsam::Pose2>           BetweenFactorPose2;
typedef gtsam::BetweenFactor<gtsam::Pose3>           BetweenFactorPose3;

typedef gtsam::NonlinearEquality<gtsam::LieVector>         NonlinearEqualityLieVector;
typedef gtsam::NonlinearEquality<gtsam::LieMatrix>         NonlinearEqualityLieMatrix;
typedef gtsam::NonlinearEquality<gtsam::Point2>            NonlinearEqualityPoint2;
typedef gtsam::NonlinearEquality<gtsam::StereoPoint2>      NonlinearEqualityStereoPoint2;
typedef gtsam::NonlinearEquality<gtsam::Point3>            NonlinearEqualityPoint3;
typedef gtsam::NonlinearEquality<gtsam::Rot2>              NonlinearEqualityRot2;
typedef gtsam::NonlinearEquality<gtsam::Rot3>              NonlinearEqualityRot3;
typedef gtsam::NonlinearEquality<gtsam::Pose2>             NonlinearEqualityPose2;
typedef gtsam::NonlinearEquality<gtsam::Pose3>             NonlinearEqualityPose3;
typedef gtsam::NonlinearEquality<gtsam::Cal3_S2>           NonlinearEqualityCal3_S2;
typedef gtsam::NonlinearEquality<gtsam::Cal3DS2>           NonlinearEqualityCal3DS2;
typedef gtsam::NonlinearEquality<gtsam::CalibratedCamera>  NonlinearEqualityCalibratedCamera;
typedef gtsam::NonlinearEquality<gtsam::SimpleCamera>      NonlinearEqualitySimpleCamera;
typedef gtsam::NonlinearEquality<gtsam::StereoCamera>      NonlinearEqualityStereoCamera;

typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> GenericProjectionFactorCal3_S2;
typedef gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2> GenericProjectionFactorCal3DS2;
typedef gtsam::GeneralSFMFactor<gtsam::SimpleCamera, gtsam::Point3> GeneralSFMFactorCal3_S2;
typedef gtsam::GeneralSFMFactor2<gtsam::Cal3_S2> GeneralSFMFactor2Cal3_S2;
typedef gtsam::GenericStereoFactor<gtsam::Pose3, gtsam::Point3> GenericStereoFactor3D;

/* Create GUIDs for Noisemodels */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Constrained, "gtsamnoiseModelConstrained");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Diagonal, "gtsamnoiseModelDiagonal");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Gaussian, "gtsamnoiseModelGaussian");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Unit, "gtsamnoiseModelUnit");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Isotropic, "gtsamnoiseModelIsotropic");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::Robust, "gtsamnoiseModelRobust");

BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Base , "gtsamnoiseModelmEstimatorBase");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Null , "gtsamnoiseModelmEstimatorNull");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Fair , "gtsamnoiseModelmEstimatorFair");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Huber, "gtsamnoiseModelmEstimatorHuber");
BOOST_CLASS_EXPORT_GUID(gtsam::noiseModel::mEstimator::Tukey, "gtsamnoiseModelmEstimatorTukey");

BOOST_CLASS_EXPORT_GUID(gtsam::SharedNoiseModel, "gtsamSharedNoiseModel");
BOOST_CLASS_EXPORT_GUID(gtsam::SharedDiagonal, "gtsamSharedDiagonal");

/* Create GUIDs for geometry */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::Point2, "gtsamPoint2");
BOOST_CLASS_EXPORT_GUID(gtsam::Point3, "gtsamPoint3");
BOOST_CLASS_EXPORT_GUID(gtsam::Rot2, "gtsamRot2");
BOOST_CLASS_EXPORT_GUID(gtsam::Rot3, "gtsamRot3");
BOOST_CLASS_EXPORT_GUID(gtsam::Pose2, "gtsamPose2");
BOOST_CLASS_EXPORT_GUID(gtsam::Pose3, "gtsamPose3");
BOOST_CLASS_EXPORT_GUID(gtsam::Cal3_S2, "gtsamCal3_S2");
BOOST_CLASS_EXPORT_GUID(gtsam::Cal3DS2, "gtsamCal3DS2");
BOOST_CLASS_EXPORT_GUID(gtsam::Cal3_S2Stereo, "gtsamCal3_S2Stereo");
BOOST_CLASS_EXPORT_GUID(gtsam::CalibratedCamera, "gtsamCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(gtsam::SimpleCamera, "gtsamSimpleCamera");
BOOST_CLASS_EXPORT_GUID(gtsam::StereoCamera, "gtsamStereoCamera");

/* Create GUIDs for factors */
/* ************************************************************************* */
BOOST_CLASS_EXPORT_GUID(gtsam::JacobianFactor, "gtsamJacobianFactor");
BOOST_CLASS_EXPORT_GUID(gtsam::HessianFactor , "gtsamHessianFactor");

BOOST_CLASS_EXPORT_GUID(PriorFactorLieVector, "gtsamPriorFactorLieVector");
BOOST_CLASS_EXPORT_GUID(PriorFactorLieMatrix, "gtsamPriorFactorLieMatrix");
BOOST_CLASS_EXPORT_GUID(PriorFactorPoint2, "gtsamPriorFactorPoint2");
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoPoint2, "gtsamPriorFactorStereoPoint2");
BOOST_CLASS_EXPORT_GUID(PriorFactorPoint3, "gtsamPriorFactorPoint3");
BOOST_CLASS_EXPORT_GUID(PriorFactorRot2, "gtsamPriorFactorRot2");
BOOST_CLASS_EXPORT_GUID(PriorFactorRot3, "gtsamPriorFactorRot3");
BOOST_CLASS_EXPORT_GUID(PriorFactorPose2, "gtsamPriorFactorPose2");
BOOST_CLASS_EXPORT_GUID(PriorFactorPose3, "gtsamPriorFactorPose3");
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3_S2, "gtsamPriorFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(PriorFactorCal3DS2, "gtsamPriorFactorCal3DS2");
BOOST_CLASS_EXPORT_GUID(PriorFactorCalibratedCamera, "gtsamPriorFactorCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(PriorFactorSimpleCamera, "gtsamPriorFactorSimpleCamera");
BOOST_CLASS_EXPORT_GUID(PriorFactorStereoCamera, "gtsamPriorFactorStereoCamera");

BOOST_CLASS_EXPORT_GUID(BetweenFactorLieVector, "gtsamBetweenFactorLieVector");
BOOST_CLASS_EXPORT_GUID(BetweenFactorLieMatrix, "gtsamBetweenFactorLieMatrix");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint2, "gtsamBetweenFactorPoint2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPoint3, "gtsamBetweenFactorPoint3");
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot2, "gtsamBetweenFactorRot2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorRot3, "gtsamBetweenFactorRot3");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPose2, "gtsamBetweenFactorPose2");
BOOST_CLASS_EXPORT_GUID(BetweenFactorPose3, "gtsamBetweenFactorPose3");

BOOST_CLASS_EXPORT_GUID(NonlinearEqualityLieVector, "gtsamNonlinearEqualityLieVector");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityLieMatrix, "gtsamNonlinearEqualityLieMatrix");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint2, "gtsamNonlinearEqualityPoint2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoPoint2, "gtsamNonlinearEqualityStereoPoint2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPoint3, "gtsamNonlinearEqualityPoint3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot2, "gtsamNonlinearEqualityRot2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityRot3, "gtsamNonlinearEqualityRot3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose2, "gtsamNonlinearEqualityPose2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityPose3, "gtsamNonlinearEqualityPose3");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3_S2, "gtsamNonlinearEqualityCal3_S2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCal3DS2, "gtsamNonlinearEqualityCal3DS2");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityCalibratedCamera, "gtsamNonlinearEqualityCalibratedCamera");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualitySimpleCamera, "gtsamNonlinearEqualitySimpleCamera");
BOOST_CLASS_EXPORT_GUID(NonlinearEqualityStereoCamera, "gtsamNonlinearEqualityStereoCamera");

BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3_S2, "gtsamGenericProjectionFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(GenericProjectionFactorCal3DS2, "gtsamGenericProjectionFactorCal3DS2");
BOOST_CLASS_EXPORT_GUID(GeneralSFMFactorCal3_S2, "gtsamGeneralSFMFactorCal3_S2");
BOOST_CLASS_EXPORT_GUID(GeneralSFMFactor2Cal3_S2, "gtsamGeneralSFMFactor2Cal3_S2");
BOOST_CLASS_EXPORT_GUID(GenericStereoFactor3D, "gtsamGenericStereoFactor3D");

// why this does not solve the problem with GTSAM serialization in different versions of Boost?
BOOST_CLASS_VERSION(gtsam::NonlinearFactorGraph, 11)
BOOST_CLASS_VERSION(gtsam::Values, 11)

#endif //GTSAM_SERIALIZATION_HELPER_H
