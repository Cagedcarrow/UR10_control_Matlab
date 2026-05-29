#include "utils.h"

#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>

namespace rtfg {

// --- math utilities ---

void fail(const std::string& id, const std::string& msg) {
  mexErrMsgIdAndTxt(id.c_str(), "%s", msg.c_str());
}

double clamp01(double x) {
  return std::max(0.0, std::min(1.0, x));
}

double rad2deg(double x) {
  return x * 180.0 / M_PI;
}

Mat4 rtToTform(const Eigen::Matrix3d& R, const Vec3& p) {
  Mat4 T = Mat4::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = p;
  return T;
}

Eigen::Matrix3d rpyToRot(double r, double p, double y) {
  Eigen::AngleAxisd rx(r, Vec3::UnitX());
  Eigen::AngleAxisd ry(p, Vec3::UnitY());
  Eigen::AngleAxisd rz(y, Vec3::UnitZ());
  return (rz * ry * rx).toRotationMatrix();
}

Mat4 xyzrpyToTform(const Vec3& xyz, const Vec3& rpy) {
  return rtToTform(rpyToRot(rpy.x(), rpy.y(), rpy.z()), xyz);
}

Vec3 rotToLogVec(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  if (std::isnan(aa.angle()) || aa.axis().hasNaN()) {
    return Vec3::Zero();
  }
  double angle = aa.angle();
  if (angle < 1e-12) {
    return Vec3::Zero();
  }
  return aa.axis() * angle;
}

double rotationDistance(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1) {
  return rotToLogVec(R0.transpose() * R1).norm();
}

double quinticBlend(double t) {
  t = clamp01(t);
  return 10.0 * t * t * t - 15.0 * t * t * t * t + 6.0 * t * t * t * t * t;
}

Eigen::Matrix3d slerpRotation(const Eigen::Matrix3d& R0, const Eigen::Matrix3d& R1, double a) {
  Eigen::Quaterniond q0(R0);
  Eigen::Quaterniond q1(R1);
  if (q0.dot(q1) < 0.0) {
    q1.coeffs() *= -1.0;
  }
  return q0.slerp(clamp01(a), q1).toRotationMatrix();
}

Eigen::VectorXd wrapJointDelta(const Eigen::VectorXd& dq) {
  Eigen::VectorXd out = dq;
  for (int i = 0; i < out.size(); ++i) {
    out[i] = std::atan2(std::sin(out[i]), std::cos(out[i]));
  }
  return out;
}

double continuityCost(const Eigen::VectorXd& q_candidate, const Eigen::VectorXd& q_prev,
                      const Eigen::VectorXd& dq_prev) {
  Eigen::VectorXd dq_pos = wrapJointDelta(q_candidate - q_prev);
  Eigen::VectorXd dq_vel = dq_pos - dq_prev;
  return dq_pos.norm() + 0.65 * dq_vel.norm();
}

Vec6 poseError(const Mat4& current, const Mat4& target) {
  Vec6 err = Vec6::Zero();
  err.head<3>() = target.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
  Eigen::Matrix3d Rerr = current.block<3, 3>(0, 0).transpose() * target.block<3, 3>(0, 0);
  Vec3 w_local = rotToLogVec(Rerr);
  err.tail<3>() = current.block<3, 3>(0, 0) * w_local;
  return err;
}

// --- mxArray conversion ---

std::string mxArrayToStdString(const mxArray* arr) {
  if (!mxIsChar(arr)) {
    fail("rtfg_solver_mex:TypeError", "Expected char array input");
  }
  char* c = mxArrayToString(arr);
  if (!c) {
    fail("rtfg_solver_mex:ConversionFailed", "Failed to convert string");
  }
  std::string out(c);
  mxFree(c);
  return out;
}

mxArray* makeStringCellArray(const std::vector<std::string>& values) {
  mxArray* out = mxCreateCellMatrix(static_cast<mwSize>(values.size()), 1);
  for (mwIndex i = 0; i < values.size(); ++i) {
    mxSetCell(out, i, mxCreateString(values[i].c_str()));
  }
  return out;
}

std::vector<std::string> readCellstrVector(const mxArray* arr) {
  if (!mxIsCell(arr)) {
    fail("rtfg_solver_mex:TypeError", "Expected cell array of strings");
  }
  std::vector<std::string> values(mxGetNumberOfElements(arr));
  for (mwIndex i = 0; i < values.size(); ++i) {
    values[i] = mxArrayToStdString(mxGetCell(arr, i));
  }
  return values;
}

Eigen::VectorXd readRowVector(const mxArray* arr) {
  if (!mxIsDouble(arr) || mxIsComplex(arr)) {
    fail("rtfg_solver_mex:TypeError", "Expected real double vector");
  }
  mwSize n = mxGetNumberOfElements(arr);
  Eigen::VectorXd out(static_cast<int>(n));
  const double* p = mxGetPr(arr);
  for (mwIndex i = 0; i < n; ++i) {
    out(static_cast<int>(i)) = p[i];
  }
  return out;
}

Eigen::MatrixXd readMatrix(const mxArray* arr) {
  if (!mxIsDouble(arr) || mxIsComplex(arr)) {
    fail("rtfg_solver_mex:TypeError", "Expected real double matrix");
  }
  mwSize m = mxGetM(arr);
  mwSize n = mxGetN(arr);
  Eigen::MatrixXd out(static_cast<int>(m), static_cast<int>(n));
  const double* p = mxGetPr(arr);
  for (mwSize j = 0; j < n; ++j) {
    for (mwSize i = 0; i < m; ++i) {
      out(static_cast<int>(i), static_cast<int>(j)) = p[j * m + i];
    }
  }
  return out;
}

std::vector<Mat4> readTformStack(const mxArray* arr) {
  if (!mxIsDouble(arr) || mxIsComplex(arr)) {
    fail("rtfg_solver_mex:TypeError", "Expected 4x4 or 4x4xN real double array");
  }
  mwSize ndims = mxGetNumberOfDimensions(arr);
  const mwSize* dims = mxGetDimensions(arr);
  if (dims[0] != 4 || dims[1] != 4) {
    fail("rtfg_solver_mex:ShapeError", "Expected 4x4xN array for target transforms");
  }
  mwSize n = (ndims >= 3) ? dims[2] : 1;
  const double* p = mxGetPr(arr);
  std::vector<Mat4> out(n, Mat4::Identity());
  for (mwSize k = 0; k < n; ++k) {
    for (mwSize c = 0; c < 4; ++c) {
      for (mwSize r = 0; r < 4; ++r) {
        out[k](static_cast<int>(r), static_cast<int>(c)) = p[k * 16 + c * 4 + r];
      }
    }
  }
  return out;
}

mxArray* eigenMatrixToMx(const Eigen::MatrixXd& M) {
  mxArray* out =
      mxCreateDoubleMatrix(static_cast<mwSize>(M.rows()), static_cast<mwSize>(M.cols()), mxREAL);
  double* p = mxGetPr(out);
  for (mwSize j = 0; j < static_cast<mwSize>(M.cols()); ++j) {
    for (mwSize i = 0; i < static_cast<mwSize>(M.rows()); ++i) {
      p[j * M.rows() + i] = M(static_cast<int>(i), static_cast<int>(j));
    }
  }
  return out;
}

Mat4 urdfPoseToTform(const urdf::Pose& pose) {
  Vec3 xyz(pose.position.x, pose.position.y, pose.position.z);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  pose.rotation.getRPY(roll, pitch, yaw);
  return xyzrpyToTform(xyz, Vec3(roll, pitch, yaw));
}

std::vector<BasinBox> readBasinBoxes(const mxArray* arr) {
  if (!mxIsStruct(arr)) {
    fail("rtfg_solver_mex:TypeError", "basinBoxes must be a struct array");
  }
  mwSize n = mxGetNumberOfElements(arr);
  std::vector<BasinBox> boxes(n);
  for (mwIndex i = 0; i < n; ++i) {
    boxes[i].name = mxArrayToStdString(mxGetField(arr, i, "name"));
    Eigen::MatrixXd sizeM = readMatrix(mxGetField(arr, i, "size"));
    if (sizeM.size() != 3) {
      fail("rtfg_solver_mex:ShapeError", "basin box size must have 3 elements");
    }
    boxes[i].size = Vec3(sizeM(0), sizeM(1), sizeM(2));
    Eigen::MatrixXd poseM = readMatrix(mxGetField(arr, i, "pose"));
    if (poseM.rows() != 4 || poseM.cols() != 4) {
      fail("rtfg_solver_mex:ShapeError", "basin box pose must be 4x4");
    }
    boxes[i].pose = poseM;
  }
  return boxes;
}

}  // namespace rtfg
