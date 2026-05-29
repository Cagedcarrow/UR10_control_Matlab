#pragma once

#include "types.h"
#include "mex.h"
#include "matrix.h"

#include <Eigen/Dense>

#include <string>
#include <vector>

// Forward declare urdf types
namespace urdf {
struct Pose;
}

namespace rtfg {

// --- math utilities ---

[[noreturn]] void fail(const std::string& id, const std::string& msg);
double clamp01(double x);
double rad2deg(double x);
Mat4 rtToTform(const Eigen::Matrix3d& R, const Vec3& p);
Eigen::Matrix3d rpyToRot(double r, double p, double y);
Mat4 xyzrpyToTform(const Vec3& xyz, const Vec3& rpy);
Vec3 rotToLogVec(const Eigen::Matrix3d& R);
double rotationDistance(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1);
double quinticBlend(double t);
Eigen::Matrix3d slerpRotation(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1, double a);
Eigen::VectorXd wrapJointDelta(const Eigen::VectorXd& dq);
double continuityCost(const Eigen::VectorXd& q_candidate, const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev);
Vec6 poseError(const Mat4& current, const Mat4& target);

// --- mxArray conversion ---

std::string mxArrayToStdString(const mxArray* arr);
mxArray* makeStringCellArray(const std::vector<std::string>& values);
std::vector<std::string> readCellstrVector(const mxArray* arr);
Eigen::VectorXd readRowVector(const mxArray* arr);
Eigen::MatrixXd readMatrix(const mxArray* arr);
std::vector<Mat4> readTformStack(const mxArray* arr);
mxArray* eigenMatrixToMx(const Eigen::MatrixXd& M);
Mat4 urdfPoseToTform(const urdf::Pose& pose);
std::vector<BasinBox> readBasinBoxes(const mxArray* arr);

}  // namespace rtfg
