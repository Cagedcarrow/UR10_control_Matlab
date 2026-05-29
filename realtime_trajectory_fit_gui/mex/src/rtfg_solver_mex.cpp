#include "mex.h"
#include "matrix.h"

#include <Eigen/Dense>

#include <fcl/fcl.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Mat4 = Eigen::Matrix4d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using CollisionObjectd = fcl::CollisionObjectd;
using CollisionGeometryd = fcl::CollisionGeometryd;

struct BasinBox {
  std::string name;
  Vec3 size;
  Mat4 pose;
};

struct ProgressEvent {
  std::string phase;
  double progress = 0.0;
  std::string message;
};

struct SegmentSpec {
  std::string joint_name;
  std::string parent_link;
  std::string child_link;
  Mat4 origin = Mat4::Identity();
  Vec3 axis = Vec3::UnitZ();
  bool movable = false;
  int q_index = -1;
  double lower = -M_PI;
  double upper = M_PI;
};

enum class GeometryType {
  Box,
  Cylinder,
  Mesh,
  Unknown
};

struct LinkCollision {
  std::string link_name;
  int chain_index = -1;
  bool is_tool = false;
  Mat4 origin = Mat4::Identity();
  GeometryType type = GeometryType::Unknown;
  Vec3 size = Vec3::Zero();
  double radius = 0.0;
  double length = 0.0;
  std::shared_ptr<CollisionGeometryd> geometry;
};

struct RobotModel {
  std::vector<SegmentSpec> segments;
  std::vector<LinkCollision> collisions;
  std::vector<std::string> link_order;
  std::unordered_map<std::string, int> link_index;
  std::vector<std::string> tool_links;
  std::string base_link;
  std::string tip_link;
  std::string mesh_root;
};

struct CandidateInfo {
  Eigen::VectorXd q;
  double pos_err = std::numeric_limits<double>::infinity();
  double rot_err = std::numeric_limits<double>::infinity();
  double clearance = -std::numeric_limits<double>::infinity();
  double cost = std::numeric_limits<double>::infinity();
  std::string failure_reason;
  bool valid = false;
};

struct CollisionSummary {
  double min_self = std::numeric_limits<double>::infinity();
  double min_tool_body = std::numeric_limits<double>::infinity();
  double min_tool_basin = std::numeric_limits<double>::infinity();
  std::string min_self_object;
  std::string min_tool_body_object;
  std::string min_tool_basin_object;
  std::string violation_type;
  std::string violation_object;
  double violation_clearance = std::numeric_limits<double>::infinity();
};

struct SolverConfig {
  double clearance_threshold = 2e-3;
  double ik_position_tolerance = 3e-2;
  int max_iterations = 200;
  double lambda = 5e-3;
};

struct TrajectoryResult {
  Eigen::MatrixXd anchor_q;
  Eigen::MatrixXd playback_q;
  Eigen::MatrixXd tcp_path;
  std::vector<std::string> playback_segment_names;
  std::vector<ProgressEvent> progress_events;
  CollisionSummary global_minimums;
  std::vector<int> collision_traj_indices;
  std::vector<int> collision_sample_indices;
  std::vector<std::string> collision_types;
  std::vector<std::string> collision_objects;
  std::vector<std::string> collision_segments;
  Eigen::MatrixXd collision_points;
  bool has_collision = false;
  int first_collision_traj_idx = -1;
  int first_collision_sample_idx = -1;
  std::string first_collision_type;
  std::string first_collision_object;
  std::string first_collision_segment;
  Vec3 first_collision_point = Vec3::Zero();
  double first_collision_clearance = std::numeric_limits<double>::infinity();
  double max_target_rot_deg = 0.0;
  double max_actual_rot_deg = 0.0;
  double max_anchor_qstep_deg = 0.0;
  double max_playback_qstep_deg = 0.0;
};

[[noreturn]] void fail(const std::string& id, const std::string& msg) {
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
  T.block<3,3>(0,0) = R;
  T.block<3,1>(0,3) = p;
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

double continuityCost(const Eigen::VectorXd& q_candidate, const Eigen::VectorXd& q_prev, const Eigen::VectorXd& dq_prev) {
  Eigen::VectorXd dq_pos = wrapJointDelta(q_candidate - q_prev);
  Eigen::VectorXd dq_vel = dq_pos - dq_prev;
  return dq_pos.norm() + 0.65 * dq_vel.norm();
}

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
  mxArray* out = mxCreateDoubleMatrix(static_cast<mwSize>(M.rows()), static_cast<mwSize>(M.cols()), mxREAL);
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

std::shared_ptr<CollisionGeometryd> loadStlAsFclModel(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("Failed to open STL: " + path);
  }

  std::vector<Vec3> vertices;
  std::vector<fcl::Triangle> triangles;

  char header[80] = {};
  in.read(header, 80);
  std::uint32_t tri_count = 0;
  in.read(reinterpret_cast<char*>(&tri_count), sizeof(tri_count));
  bool binary_ok = in.good();
  if (binary_ok) {
    in.seekg(0, std::ios::end);
    std::streamoff file_size = in.tellg();
    std::streamoff expected = 84 + static_cast<std::streamoff>(tri_count) * 50;
    binary_ok = (file_size == expected);
  }

  if (binary_ok) {
    in.seekg(84, std::ios::beg);
    vertices.reserve(static_cast<size_t>(tri_count) * 3);
    triangles.reserve(static_cast<size_t>(tri_count));
    for (std::uint32_t i = 0; i < tri_count; ++i) {
      float buf[12];
      std::uint16_t attr = 0;
      in.read(reinterpret_cast<char*>(buf), sizeof(buf));
      in.read(reinterpret_cast<char*>(&attr), sizeof(attr));
      if (!in.good()) {
        throw std::runtime_error("Corrupt binary STL: " + path);
      }
      Vec3 v0(buf[3], buf[4], buf[5]);
      Vec3 v1(buf[6], buf[7], buf[8]);
      Vec3 v2(buf[9], buf[10], buf[11]);
      int base = static_cast<int>(vertices.size());
      vertices.push_back(v0);
      vertices.push_back(v1);
      vertices.push_back(v2);
      triangles.emplace_back(base, base + 1, base + 2);
    }
  } else {
    in.close();
    std::ifstream txt(path);
    if (!txt) {
      throw std::runtime_error("Failed to reopen STL as text: " + path);
    }
    std::string line;
    std::vector<Vec3> facet_vertices;
    while (std::getline(txt, line)) {
      std::istringstream iss(line);
      std::string tok;
      iss >> tok;
      if (tok == "vertex") {
        double x = 0.0, y = 0.0, z = 0.0;
        iss >> x >> y >> z;
        facet_vertices.emplace_back(x, y, z);
        if (facet_vertices.size() == 3) {
          int base = static_cast<int>(vertices.size());
          vertices.push_back(facet_vertices[0]);
          vertices.push_back(facet_vertices[1]);
          vertices.push_back(facet_vertices[2]);
          triangles.emplace_back(base, base + 1, base + 2);
          facet_vertices.clear();
        }
      }
    }
  }

  using BVH = fcl::BVHModel<fcl::OBBRSSd>;
  auto model = std::make_shared<BVH>();
  model->beginModel();
  model->addSubModel(vertices, triangles);
  model->endModel();
  return model;
}

std::shared_ptr<CollisionGeometryd> buildGeometryFromUrdf(const urdf::CollisionSharedPtr& collision, const std::string& mesh_root) {
  if (!collision || !collision->geometry) {
    return nullptr;
  }
  switch (collision->geometry->type) {
    case urdf::Geometry::BOX: {
      auto* box = dynamic_cast<urdf::Box*>(collision->geometry.get());
      return std::make_shared<fcl::Boxd>(box->dim.x, box->dim.y, box->dim.z);
    }
    case urdf::Geometry::CYLINDER: {
      auto* cyl = dynamic_cast<urdf::Cylinder*>(collision->geometry.get());
      return std::make_shared<fcl::Cylinderd>(cyl->radius, cyl->length);
    }
    case urdf::Geometry::MESH: {
      auto* mesh = dynamic_cast<urdf::Mesh*>(collision->geometry.get());
      std::string filename = mesh->filename;
      if (filename.rfind("package://", 0) == 0) {
        throw std::runtime_error("package:// mesh paths are not supported in rtfg_solver_mex");
      }
      std::string full_path = filename;
      if (!filename.empty() && filename[0] != '/') {
        full_path = mesh_root + "/" + filename;
      }
      return loadStlAsFclModel(full_path);
    }
    default:
      return nullptr;
  }
}

RobotModel loadRobotModel(const std::string& urdf_path, const std::string& base_link, const std::string& tip_link) {
  auto model = urdf::parseURDFFile(urdf_path);
  if (!model) {
    throw std::runtime_error("Failed to parse URDF: " + urdf_path);
  }

  RobotModel robot;
  robot.base_link = base_link;
  robot.tip_link = tip_link;
  std::string mesh_root = urdf_path.substr(0, urdf_path.find_last_of('/'));
  robot.mesh_root = mesh_root;

  std::unordered_map<std::string, urdf::JointSharedPtr> child_joint_map;
  for (const auto& kv : model->joints_) {
    child_joint_map[kv.second->child_link_name] = kv.second;
  }

  std::vector<urdf::JointSharedPtr> reverse_joints;
  std::string current = tip_link;
  while (current != base_link) {
    auto it = child_joint_map.find(current);
    if (it == child_joint_map.end()) {
      throw std::runtime_error("Unable to trace chain from tip to base for link: " + current);
    }
    reverse_joints.push_back(it->second);
    current = it->second->parent_link_name;
  }
  std::reverse(reverse_joints.begin(), reverse_joints.end());

  robot.link_order.push_back(base_link);
  robot.link_index[base_link] = 0;
  int movable_idx = 0;
  for (size_t i = 0; i < reverse_joints.size(); ++i) {
    const auto& joint = reverse_joints[i];
    SegmentSpec seg;
    seg.joint_name = joint->name;
    seg.parent_link = joint->parent_link_name;
    seg.child_link = joint->child_link_name;
    seg.origin = urdfPoseToTform(joint->parent_to_joint_origin_transform);
    seg.axis = Vec3(joint->axis.x, joint->axis.y, joint->axis.z);
    if (seg.axis.norm() < 1e-12) {
      seg.axis = Vec3::UnitZ();
    } else {
      seg.axis.normalize();
    }
    seg.movable = (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS);
    if (seg.movable) {
      seg.q_index = movable_idx++;
      if (joint->type == urdf::Joint::CONTINUOUS || !joint->limits) {
        seg.lower = -2.0 * M_PI;
        seg.upper = 2.0 * M_PI;
      } else {
        seg.lower = joint->limits->lower;
        seg.upper = joint->limits->upper;
      }
    }
    robot.segments.push_back(seg);
    robot.link_index[seg.child_link] = static_cast<int>(robot.link_order.size());
    robot.link_order.push_back(seg.child_link);
  }

  std::set<std::string> tool_names = {"sensor_shovel", "sensor_shovel_tcp"};
  for (size_t i = 0; i < robot.link_order.size(); ++i) {
    auto link = model->getLink(robot.link_order[i]);
    if (!link || !link->collision) {
      continue;
    }
    auto geometry = buildGeometryFromUrdf(link->collision, mesh_root);
    if (!geometry) {
      continue;
    }
    LinkCollision lc;
    lc.link_name = link->name;
    lc.chain_index = static_cast<int>(i);
    lc.is_tool = tool_names.count(link->name) > 0;
    lc.origin = urdfPoseToTform(link->collision->origin);
    lc.geometry = geometry;
    if (auto* box = dynamic_cast<urdf::Box*>(link->collision->geometry.get())) {
      lc.type = GeometryType::Box;
      lc.size = Vec3(box->dim.x, box->dim.y, box->dim.z);
    } else if (auto* cyl = dynamic_cast<urdf::Cylinder*>(link->collision->geometry.get())) {
      lc.type = GeometryType::Cylinder;
      lc.radius = cyl->radius;
      lc.length = cyl->length;
    } else if (dynamic_cast<urdf::Mesh*>(link->collision->geometry.get())) {
      lc.type = GeometryType::Mesh;
    }
    robot.collisions.push_back(lc);
  }

  robot.tool_links = {"sensor_shovel", "sensor_shovel_tcp"};
  if (movable_idx != 6) {
    throw std::runtime_error("Expected UR10 six revolute joints, got " + std::to_string(movable_idx));
  }
  return robot;
}

std::unordered_map<std::string, Mat4> forwardKinematics(const RobotModel& robot, const Eigen::VectorXd& q) {
  std::unordered_map<std::string, Mat4> poses;
  poses[robot.base_link] = Mat4::Identity();
  Mat4 T = Mat4::Identity();
  for (const auto& seg : robot.segments) {
    T = T * seg.origin;
    if (seg.movable) {
      Eigen::AngleAxisd aa(q(seg.q_index), seg.axis);
      T = T * rtToTform(aa.toRotationMatrix(), Vec3::Zero());
    }
    poses[seg.child_link] = T;
  }
  return poses;
}

Mat4 tipTransform(const RobotModel& robot, const Eigen::VectorXd& q) {
  auto poses = forwardKinematics(robot, q);
  return poses.at(robot.tip_link);
}

Vec6 poseError(const Mat4& current, const Mat4& target) {
  Vec6 err = Vec6::Zero();
  err.head<3>() = target.block<3,1>(0,3) - current.block<3,1>(0,3);
  Eigen::Matrix3d Rerr = current.block<3,3>(0,0).transpose() * target.block<3,3>(0,0);
  Vec3 w_local = rotToLogVec(Rerr);
  err.tail<3>() = current.block<3,3>(0,0) * w_local;
  return err;
}

Mat6 numericJacobian(const RobotModel& robot, const Eigen::VectorXd& q) {
  const double eps = 1e-6;
  Mat6 J = Mat6::Zero();
  Mat4 T0 = tipTransform(robot, q);
  for (int i = 0; i < q.size(); ++i) {
    Eigen::VectorXd qp = q;
    qp(i) += eps;
    Mat4 Tp = tipTransform(robot, qp);
    Vec6 delta = poseError(T0, Tp);
    J.col(i) = -delta / eps;
  }
  return J;
}

Eigen::VectorXd clampToLimits(const RobotModel& robot, const Eigen::VectorXd& q) {
  Eigen::VectorXd qc = q;
  for (const auto& seg : robot.segments) {
    if (!seg.movable) {
      continue;
    }
    qc(seg.q_index) = std::min(std::max(qc(seg.q_index), seg.lower), seg.upper);
  }
  return qc;
}

CollisionSummary evaluateConfiguration(const RobotModel& robot,
                                       const std::vector<BasinBox>& basin_boxes,
                                       const SolverConfig& cfg,
                                       const Eigen::VectorXd& q) {
  CollisionSummary summary;
  auto poses = forwardKinematics(robot, q);

  std::vector<std::unique_ptr<CollisionObjectd>> robot_objects;
  robot_objects.reserve(robot.collisions.size());
  for (const auto& col : robot.collisions) {
    Mat4 T = poses.at(col.link_name) * col.origin;
    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.matrix() = T;
    auto obj = std::make_unique<CollisionObjectd>(col.geometry, tf);
    obj->computeAABB();
    robot_objects.push_back(std::move(obj));
  }

  auto choose_violation = [&](const std::string& type, const std::string& name, double clearance) {
    static const std::map<std::string, int> pri = {{"tool_basin", 3}, {"tool_body", 2}, {"self", 1}};
    if (summary.violation_type.empty() || pri.at(type) > pri.at(summary.violation_type)) {
      summary.violation_type = type;
      summary.violation_object = name;
      summary.violation_clearance = clearance;
    }
  };

  for (size_t i = 0; i < robot.collisions.size(); ++i) {
    for (size_t j = i + 1; j < robot.collisions.size(); ++j) {
      const auto& a = robot.collisions[i];
      const auto& b = robot.collisions[j];
      if (std::abs(a.chain_index - b.chain_index) <= 1) {
        continue;
      }
      fcl::DistanceRequestd request(true);
      fcl::DistanceResultd result;
      double dist = fcl::distance(robot_objects[i].get(), robot_objects[j].get(), request, result);
      bool a_tool = a.is_tool;
      bool b_tool = b.is_tool;
      std::string pair_name = a.link_name + " <-> " + b.link_name;
      if (a_tool ^ b_tool) {
        if (dist < summary.min_tool_body) {
          summary.min_tool_body = dist;
          summary.min_tool_body_object = pair_name;
        }
        if (dist < cfg.clearance_threshold) {
          choose_violation("tool_body", pair_name, dist);
        }
      } else if (!a_tool && !b_tool) {
        if (dist < summary.min_self) {
          summary.min_self = dist;
          summary.min_self_object = pair_name;
        }
        if (dist < cfg.clearance_threshold) {
          choose_violation("self", pair_name, dist);
        }
      }
    }
  }

  std::vector<int> tool_indices;
  for (size_t i = 0; i < robot.collisions.size(); ++i) {
    if (robot.collisions[i].is_tool) {
      tool_indices.push_back(static_cast<int>(i));
    }
  }

  for (int tool_idx : tool_indices) {
    for (const auto& box : basin_boxes) {
      auto geom = std::make_shared<fcl::Boxd>(box.size.x(), box.size.y(), box.size.z());
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.matrix() = box.pose;
      CollisionObjectd basin_obj(geom, tf);
      basin_obj.computeAABB();
      fcl::DistanceRequestd request(true);
      fcl::DistanceResultd result;
      double dist = fcl::distance(robot_objects[tool_idx].get(), &basin_obj, request, result);
      std::string pair_name = box.name;
      if (dist < summary.min_tool_basin) {
        summary.min_tool_basin = dist;
        summary.min_tool_basin_object = pair_name;
      }
      if (dist < cfg.clearance_threshold) {
        choose_violation("tool_basin", pair_name, dist);
      }
    }
  }

  return summary;
}

CandidateInfo solveSinglePose(const RobotModel& robot,
                              const std::vector<BasinBox>& basin_boxes,
                              const SolverConfig& cfg,
                              const Mat4& target,
                              const Eigen::VectorXd& seed,
                              const Eigen::VectorXd& q_prev,
                              const Eigen::VectorXd& dq_prev,
                              const std::array<double, 6>& weights,
                              double orient_limit) {
  Eigen::VectorXd q = clampToLimits(robot, seed);
  CandidateInfo cand;
  for (int iter = 0; iter < cfg.max_iterations; ++iter) {
    Mat4 T = tipTransform(robot, q);
    Vec6 err = poseError(T, target);
    Vec6 weighted = err;
    for (int k = 0; k < 6; ++k) {
      weighted(k) *= weights[k];
    }
    if (err.head<3>().norm() <= cfg.ik_position_tolerance && err.tail<3>().norm() <= orient_limit) {
      break;
    }
    Mat6 J = numericJacobian(robot, q);
    for (int r = 0; r < 6; ++r) {
      J.row(r) *= weights[r];
    }
    Eigen::MatrixXd H = J.transpose() * J + cfg.lambda * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd g = J.transpose() * weighted;
    Eigen::VectorXd dq = H.ldlt().solve(g);
    double step_norm = dq.norm();
    if (step_norm > 0.45) {
      dq *= (0.45 / step_norm);
    }
    q = clampToLimits(robot, q + dq);
  }

  Mat4 T = tipTransform(robot, q);
  Vec6 err = poseError(T, target);
  auto collision = evaluateConfiguration(robot, basin_boxes, cfg, q);
  double clearance = std::min({collision.min_self, collision.min_tool_body, collision.min_tool_basin});

  cand.q = q;
  cand.pos_err = err.head<3>().norm();
  cand.rot_err = err.tail<3>().norm();
  cand.clearance = clearance;
  cand.cost = continuityCost(q, q_prev, dq_prev);
  cand.failure_reason = !collision.violation_object.empty() ? collision.violation_object : std::string();
  cand.valid = true;
  return cand;
}

std::vector<Eigen::VectorXd> buildSeedList(const Eigen::VectorXd& q_prev, const Eigen::VectorXd& home_q, const RobotModel& robot) {
  std::vector<Eigen::VectorXd> seeds;
  seeds.push_back(q_prev);
  seeds.push_back(home_q);
  seeds.push_back(Eigen::VectorXd::Zero(q_prev.size()));
  std::array<double, 3> wraps = {-2.0 * M_PI, 0.0, 2.0 * M_PI};
  std::vector<Eigen::VectorXd> expanded = seeds;
  for (const auto& q0 : seeds) {
    for (double w5 : wraps) {
      for (double w6 : wraps) {
        if (w5 == 0.0 && w6 == 0.0) {
          continue;
        }
        Eigen::VectorXd q = q0;
        if (q.size() >= 5) {
          q(4) += w5;
        }
        if (q.size() >= 6) {
          q(5) += w6;
        }
        expanded.push_back(clampToLimits(robot, q));
      }
    }
  }
  std::vector<Eigen::VectorXd> uniq;
  for (const auto& q : expanded) {
    bool duplicate = false;
    for (const auto& u : uniq) {
      if ((wrapJointDelta(q - u)).norm() < 1e-9) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      uniq.push_back(q);
    }
  }
  return uniq;
}

std::vector<Eigen::VectorXd> buildGlobalSeedList(const RobotModel& robot) {
  std::vector<Eigen::VectorXd> seeds;
  Eigen::VectorXd qmid(6);
  for (const auto& seg : robot.segments) {
    if (!seg.movable) {
      continue;
    }
    qmid(seg.q_index) = 0.5 * (seg.lower + seg.upper);
  }
  seeds.push_back(qmid);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  for (int s = 0; s < 96; ++s) {
    Eigen::VectorXd q(6);
    for (const auto& seg : robot.segments) {
      if (!seg.movable) {
        continue;
      }
      q(seg.q_index) = seg.lower + (seg.upper - seg.lower) * unif(rng);
    }
    seeds.push_back(q);
  }
  return seeds;
}

TrajectoryResult solveTrajectory(const RobotModel& robot,
                                 const std::vector<BasinBox>& basin_boxes,
                                 const SolverConfig& cfg,
                                 const std::vector<Mat4>& target_tforms,
                                 const std::vector<std::string>& segment_names,
                                 const Eigen::VectorXd& current_q,
                                 const Eigen::VectorXd& home_q) {
  if (target_tforms.empty()) {
    throw std::runtime_error("Empty target trajectory");
  }

  const std::vector<std::pair<std::array<double, 6>, double>> weight_schedule = {
      {{{1, 1, 1, 0.20, 0.20, 0.20}}, M_PI / 6.0},
      {{{1, 1, 1, 0.10, 0.10, 0.10}}, M_PI / 4.0},
      {{{1, 1, 1, 0.03, 0.03, 0.03}}, 70.0 * M_PI / 180.0},
      {{{1, 1, 1, 0.00, 0.00, 0.00}}, std::numeric_limits<double>::infinity()}};

  TrajectoryResult out;
  const int n = static_cast<int>(target_tforms.size());
  out.progress_events.push_back({"模型与场景预处理", 0.05, "MEX 已开始处理轨迹求解"});

  Eigen::MatrixXd anchor_q(n, 6);
  Eigen::VectorXd q_prev = current_q;
  Eigen::VectorXd dq_prev = Eigen::VectorXd::Zero(6);

  for (int i = 0; i < n; ++i) {
    std::vector<Eigen::VectorXd> seeds = buildSeedList(q_prev, home_q, robot);
    CandidateInfo best_safe;
    CandidateInfo best_fallback;
    auto run_seed_pool = [&](const std::vector<Eigen::VectorXd>& seed_pool) {
      for (const auto& schedule : weight_schedule) {
        for (const auto& seed : seed_pool) {
          CandidateInfo cand = solveSinglePose(robot, basin_boxes, cfg, target_tforms[i], seed, q_prev, dq_prev, schedule.first, schedule.second);
          if (!best_fallback.valid || cand.pos_err < best_fallback.pos_err ||
              (std::abs(cand.pos_err - best_fallback.pos_err) < 1e-12 && cand.cost < best_fallback.cost)) {
            best_fallback = cand;
          }
          if (cand.pos_err <= cfg.ik_position_tolerance &&
              cand.rot_err <= schedule.second &&
              cand.clearance >= cfg.clearance_threshold) {
            if (!best_safe.valid || cand.cost < best_safe.cost ||
                (std::abs(cand.cost - best_safe.cost) < 1e-12 && cand.clearance > best_safe.clearance)) {
              best_safe = cand;
            }
          }
        }
        if (best_safe.valid) {
          break;
        }
      }
    };
    run_seed_pool(seeds);
    if (!best_safe.valid) {
      run_seed_pool(buildGlobalSeedList(robot));
    }
    if (!best_safe.valid) {
      std::ostringstream oss;
      oss << "第 " << (i + 1) << "/" << n << " 个目标位姿 MEX 求解失败";
      if (!segment_names.empty()) {
        oss << " [" << segment_names[i] << "]";
      }
      if (best_fallback.valid) {
        oss << "，位置误差 " << best_fallback.pos_err << " m";
        if (!best_fallback.failure_reason.empty()) {
          oss << "，clearance 违规对象: " << best_fallback.failure_reason;
        }
      }
      throw std::runtime_error(oss.str());
    }
    anchor_q.row(i) = best_safe.q.transpose();
    if (i > 0) {
      out.max_anchor_qstep_deg = std::max(out.max_anchor_qstep_deg, rad2deg((wrapJointDelta(best_safe.q - q_prev)).norm()));
    }
    dq_prev = wrapJointDelta(best_safe.q - q_prev);
    q_prev = best_safe.q;
    if (i == 0 || i == n - 1 || ((i + 1) % 10) == 0) {
      std::ostringstream oss;
      oss << "MEX 正在求解轨迹锚点 " << (i + 1) << "/" << n;
      out.progress_events.push_back({"anchor 轨迹求解", 0.05 + 0.45 * ((i + 1.0) / n), oss.str()});
    }
  }
  out.anchor_q = anchor_q;

  std::vector<Eigen::VectorXd> playback_q_list;
  std::vector<std::string> playback_segment_names;
  playback_q_list.reserve(n * 3);
  playback_segment_names.reserve(n * 3);
  Eigen::VectorXd q0 = anchor_q.row(0).transpose();
  playback_q_list.push_back(q0);
  playback_segment_names.push_back(segment_names.empty() ? std::string() : segment_names[0]);
  for (int i = 1; i < n; ++i) {
    Eigen::VectorXd qa = anchor_q.row(i - 1).transpose();
    Eigen::VectorXd qb = anchor_q.row(i).transpose();
    Mat4 Ta = tipTransform(robot, qa);
    Mat4 Tb = tipTransform(robot, qb);
    double joint_step = wrapJointDelta(qb - qa).norm();
    double pos_step = (Tb.block<3,1>(0,3) - Ta.block<3,1>(0,3)).norm();
    double rot_step = rotationDistance(Ta.block<3,3>(0,0), Tb.block<3,3>(0,0));
    int nseg = 1 + std::max({2, static_cast<int>(std::ceil(joint_step / (0.70 * M_PI / 180.0))),
                                static_cast<int>(std::ceil(pos_step / 0.0030)),
                                static_cast<int>(std::ceil(rot_step / (0.30 * M_PI / 180.0)))});
    nseg = std::min(std::max(nseg, 4), 32);
    for (int k = 1; k <= nseg; ++k) {
      double a = quinticBlend(static_cast<double>(k) / static_cast<double>(nseg));
      Eigen::VectorXd q = qa + a * wrapJointDelta(qb - qa);
      playback_q_list.push_back(q);
      playback_segment_names.push_back(segment_names.empty() ? std::string() : segment_names[i]);
    }
    if (i == n - 1 || (i % 15) == 0) {
      std::ostringstream oss;
      oss << "MEX 正在生成 playback 轨迹 " << i << "/" << (n - 1);
      out.progress_events.push_back({"playback 连续化", 0.55 + 0.20 * (static_cast<double>(i) / std::max(1, n - 1)), oss.str()});
    }
  }

  out.playback_q.resize(static_cast<int>(playback_q_list.size()), 6);
  for (int i = 0; i < static_cast<int>(playback_q_list.size()); ++i) {
    out.playback_q.row(i) = playback_q_list[i].transpose();
    if (i > 0) {
      out.max_playback_qstep_deg = std::max(out.max_playback_qstep_deg,
          rad2deg((wrapJointDelta(playback_q_list[i] - playback_q_list[i - 1])).norm()));
    }
  }
  out.playback_segment_names = playback_segment_names;

  out.tcp_path.resize(out.playback_q.rows(), 3);
  std::vector<Vec3> collision_points;
  for (int i = 0; i < out.playback_q.rows(); ++i) {
    Mat4 T = tipTransform(robot, out.playback_q.row(i).transpose());
    out.tcp_path.row(i) = T.block<3,1>(0,3).transpose();
    if (i > 0) {
      out.max_actual_rot_deg = std::max(out.max_actual_rot_deg,
          rad2deg(rotationDistance(
              tipTransform(robot, out.playback_q.row(i - 1).transpose()).block<3,3>(0,0),
              T.block<3,3>(0,0))));
    }
    CollisionSummary cm = evaluateConfiguration(robot, basin_boxes, cfg, out.playback_q.row(i).transpose());
    if (cm.min_self < out.global_minimums.min_self) {
      out.global_minimums.min_self = cm.min_self;
      out.global_minimums.min_self_object = cm.min_self_object;
    }
    if (cm.min_tool_body < out.global_minimums.min_tool_body) {
      out.global_minimums.min_tool_body = cm.min_tool_body;
      out.global_minimums.min_tool_body_object = cm.min_tool_body_object;
    }
    if (cm.min_tool_basin < out.global_minimums.min_tool_basin) {
      out.global_minimums.min_tool_basin = cm.min_tool_basin;
      out.global_minimums.min_tool_basin_object = cm.min_tool_basin_object;
    }
    if (!cm.violation_type.empty()) {
      out.has_collision = true;
      out.collision_sample_indices.push_back(i + 1);
      out.collision_traj_indices.push_back(std::min(i + 1, n));
      out.collision_types.push_back(cm.violation_type);
      out.collision_objects.push_back(cm.violation_object);
      out.collision_segments.push_back(playback_segment_names[i]);
      collision_points.push_back(T.block<3,1>(0,3));
      if (out.first_collision_sample_idx < 0) {
        out.first_collision_sample_idx = i + 1;
        out.first_collision_traj_idx = std::min(i + 1, n);
        out.first_collision_type = cm.violation_type;
        out.first_collision_object = cm.violation_object;
        out.first_collision_segment = playback_segment_names[i];
        out.first_collision_point = T.block<3,1>(0,3);
        out.first_collision_clearance = cm.violation_clearance;
      }
    }
    if (i == out.playback_q.rows() - 1 || ((i + 1) % 50) == 0) {
      std::ostringstream oss;
      oss << "MEX 正在进行碰撞复检 " << (i + 1) << "/" << out.playback_q.rows();
      out.progress_events.push_back({"碰撞复检", 0.75 + 0.20 * ((i + 1.0) / out.playback_q.rows()), oss.str()});
    }
  }
  out.collision_points.resize(static_cast<int>(collision_points.size()), 3);
  for (int i = 0; i < static_cast<int>(collision_points.size()); ++i) {
    out.collision_points.row(i) = collision_points[i].transpose();
  }
  for (int i = 1; i < n; ++i) {
    out.max_target_rot_deg = std::max(out.max_target_rot_deg,
        rad2deg(rotationDistance(target_tforms[i - 1].block<3,3>(0,0), target_tforms[i].block<3,3>(0,0))));
  }
  out.progress_events.push_back({"结果回传", 1.0, "MEX 求解完成"});
  return out;
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

mxArray* buildCollisionResultsStruct(const TrajectoryResult& result) {
  const char* fields[] = {
      "hasCollision", "collisionPoints", "collisionTypes", "collisionObjects", "segmentNames",
      "sampleIndices", "trajectoryIndices", "hitBasinNames", "firstCollision",
      "minSelfClearance", "minToolBodyClearance", "minToolBasinClearance",
      "minSelfObject", "minToolBodyObject", "minToolBasinObject",
      "minSelfTrajectoryIndex", "minToolBodyTrajectoryIndex", "minToolBasinTrajectoryIndex",
      "minSelfSegmentName", "minToolBodySegmentName", "minToolBasinSegmentName"};
  mxArray* out = mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
  mxSetField(out, 0, "hasCollision", mxCreateLogicalScalar(result.has_collision));
  mxSetField(out, 0, "collisionPoints", eigenMatrixToMx(result.collision_points));
  mxSetField(out, 0, "collisionTypes", makeStringCellArray(result.collision_types));
  mxSetField(out, 0, "collisionObjects", makeStringCellArray(result.collision_objects));
  mxSetField(out, 0, "segmentNames", makeStringCellArray(result.collision_segments));

  mxArray* sampleIdx = mxCreateDoubleMatrix(result.collision_sample_indices.size(), 1, mxREAL);
  mxArray* trajIdx = mxCreateDoubleMatrix(result.collision_traj_indices.size(), 1, mxREAL);
  for (mwIndex i = 0; i < result.collision_sample_indices.size(); ++i) {
    mxGetPr(sampleIdx)[i] = static_cast<double>(result.collision_sample_indices[i]);
    mxGetPr(trajIdx)[i] = static_cast<double>(result.collision_traj_indices[i]);
  }
  mxSetField(out, 0, "sampleIndices", sampleIdx);
  mxSetField(out, 0, "trajectoryIndices", trajIdx);

  std::vector<std::string> basin_hits;
  for (size_t i = 0; i < result.collision_types.size(); ++i) {
    if (result.collision_types[i] == "tool_basin") {
      basin_hits.push_back(result.collision_objects[i]);
    }
  }
  std::sort(basin_hits.begin(), basin_hits.end());
  basin_hits.erase(std::unique(basin_hits.begin(), basin_hits.end()), basin_hits.end());
  mxSetField(out, 0, "hitBasinNames", makeStringCellArray(basin_hits));

  const char* fcFields[] = {"sampleIndex", "trajectoryIndex", "segmentName", "type", "objectName", "point", "clearance"};
  mxArray* fc = mxCreateStructMatrix(result.first_collision_sample_idx >= 0 ? 1 : 0, 1, sizeof(fcFields) / sizeof(fcFields[0]), fcFields);
  if (result.first_collision_sample_idx >= 0) {
    mxSetField(fc, 0, "sampleIndex", mxCreateDoubleScalar(result.first_collision_sample_idx));
    mxSetField(fc, 0, "trajectoryIndex", mxCreateDoubleScalar(result.first_collision_traj_idx));
    mxSetField(fc, 0, "segmentName", mxCreateString(result.first_collision_segment.c_str()));
    mxSetField(fc, 0, "type", mxCreateString(result.first_collision_type.c_str()));
    mxSetField(fc, 0, "objectName", mxCreateString(result.first_collision_object.c_str()));
    Eigen::Matrix<double, 1, 3> pt = result.first_collision_point.transpose();
    mxSetField(fc, 0, "point", eigenMatrixToMx(pt));
    mxSetField(fc, 0, "clearance", mxCreateDoubleScalar(result.first_collision_clearance));
  }
  mxSetField(out, 0, "firstCollision", fc);

  mxSetField(out, 0, "minSelfClearance", mxCreateDoubleScalar(result.global_minimums.min_self));
  mxSetField(out, 0, "minToolBodyClearance", mxCreateDoubleScalar(result.global_minimums.min_tool_body));
  mxSetField(out, 0, "minToolBasinClearance", mxCreateDoubleScalar(result.global_minimums.min_tool_basin));
  mxSetField(out, 0, "minSelfObject", mxCreateString(result.global_minimums.min_self_object.c_str()));
  mxSetField(out, 0, "minToolBodyObject", mxCreateString(result.global_minimums.min_tool_body_object.c_str()));
  mxSetField(out, 0, "minToolBasinObject", mxCreateString(result.global_minimums.min_tool_basin_object.c_str()));
  mxSetField(out, 0, "minSelfTrajectoryIndex", mxCreateDoubleScalar(std::numeric_limits<double>::quiet_NaN()));
  mxSetField(out, 0, "minToolBodyTrajectoryIndex", mxCreateDoubleScalar(std::numeric_limits<double>::quiet_NaN()));
  mxSetField(out, 0, "minToolBasinTrajectoryIndex", mxCreateDoubleScalar(std::numeric_limits<double>::quiet_NaN()));
  mxSetField(out, 0, "minSelfSegmentName", mxCreateString(""));
  mxSetField(out, 0, "minToolBodySegmentName", mxCreateString(""));
  mxSetField(out, 0, "minToolBasinSegmentName", mxCreateString(""));
  return out;
}

mxArray* buildMetricsStruct(const TrajectoryResult& result) {
  const char* fields[] = {
      "anchorCount", "playbackCount", "maxTargetRotationDeltaDeg", "maxActualRotationDeltaDeg",
      "maxAnchorJointStepDegNorm", "maxPlaybackJointStepDegNorm", "moveToTrackTcpDeltaDeg",
      "moveToTrackJointDeltaDegNorm", "anchorSampleIndices"};
  mxArray* out = mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
  mxSetField(out, 0, "anchorCount", mxCreateDoubleScalar(result.anchor_q.rows()));
  mxSetField(out, 0, "playbackCount", mxCreateDoubleScalar(result.playback_q.rows()));
  mxSetField(out, 0, "maxTargetRotationDeltaDeg", mxCreateDoubleScalar(result.max_target_rot_deg));
  mxSetField(out, 0, "maxActualRotationDeltaDeg", mxCreateDoubleScalar(result.max_actual_rot_deg));
  mxSetField(out, 0, "maxAnchorJointStepDegNorm", mxCreateDoubleScalar(result.max_anchor_qstep_deg));
  mxSetField(out, 0, "maxPlaybackJointStepDegNorm", mxCreateDoubleScalar(result.max_playback_qstep_deg));
  mxSetField(out, 0, "moveToTrackTcpDeltaDeg", mxCreateDoubleScalar(0.0));
  mxSetField(out, 0, "moveToTrackJointDeltaDegNorm", mxCreateDoubleScalar(0.0));
  mxSetField(out, 0, "anchorSampleIndices", mxCreateDoubleMatrix(0, 0, mxREAL));
  return out;
}

mxArray* buildProgressStruct(const std::vector<ProgressEvent>& events) {
  const char* fields[] = {"phase", "progress", "message"};
  mxArray* out = mxCreateStructMatrix(events.size(), 1, sizeof(fields) / sizeof(fields[0]), fields);
  for (mwIndex i = 0; i < events.size(); ++i) {
    mxSetField(out, i, "phase", mxCreateString(events[i].phase.c_str()));
    mxSetField(out, i, "progress", mxCreateDoubleScalar(events[i].progress));
    mxSetField(out, i, "message", mxCreateString(events[i].message.c_str()));
  }
  return out;
}

}  // namespace

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  if (nrhs != 1 || !mxIsStruct(prhs[0])) {
    fail("rtfg_solver_mex:InvalidInput", "Usage: out = rtfg_solver_mex(inputStruct)");
  }
  if (nlhs > 1) {
    fail("rtfg_solver_mex:InvalidOutput", "rtfg_solver_mex returns one struct output");
  }

  try {
    const mxArray* in = prhs[0];
    auto getField = [&](const char* name) -> const mxArray* {
      const mxArray* f = mxGetField(in, 0, name);
      if (!f) {
        fail("rtfg_solver_mex:MissingField", std::string("Missing input field: ") + name);
      }
      return f;
    };

    std::string scene_urdf = mxArrayToStdString(getField("sceneUrdf"));
    std::string base_link = mxArrayToStdString(getField("baseLink"));
    std::string tip_link = mxArrayToStdString(getField("tipLink"));
    Eigen::VectorXd current_q = readRowVector(getField("currentQ"));
    Eigen::VectorXd initial_q = readRowVector(getField("initialQ"));
    std::vector<Mat4> target_tforms = readTformStack(getField("targetTforms"));
    std::vector<std::string> segment_names = readCellstrVector(getField("segmentNames"));
    std::vector<BasinBox> basin_boxes = readBasinBoxes(getField("basinBoxes"));
    double clearance_threshold = mxGetScalar(getField("clearanceThreshold"));

    SolverConfig cfg;
    cfg.clearance_threshold = clearance_threshold;

    RobotModel robot = loadRobotModel(scene_urdf, base_link, tip_link);
    TrajectoryResult result = solveTrajectory(robot, basin_boxes, cfg, target_tforms, segment_names, current_q, initial_q);

    const char* fields[] = {
        "anchorQSeries", "playbackQSeries", "tcpPath",
        "collisionResults", "previewMetrics", "progressEvents"};
    plhs[0] = mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
    mxSetField(plhs[0], 0, "anchorQSeries", eigenMatrixToMx(result.anchor_q));
    mxSetField(plhs[0], 0, "playbackQSeries", eigenMatrixToMx(result.playback_q));
    mxSetField(plhs[0], 0, "tcpPath", eigenMatrixToMx(result.tcp_path));
    mxSetField(plhs[0], 0, "collisionResults", buildCollisionResultsStruct(result));
    mxSetField(plhs[0], 0, "previewMetrics", buildMetricsStruct(result));
    mxSetField(plhs[0], 0, "progressEvents", buildProgressStruct(result.progress_events));
  } catch (const std::exception& ex) {
    fail("rtfg_solver_mex:RuntimeError", ex.what());
  } catch (...) {
    fail("rtfg_solver_mex:UnknownError", "Unknown exception in rtfg_solver_mex");
  }
}
