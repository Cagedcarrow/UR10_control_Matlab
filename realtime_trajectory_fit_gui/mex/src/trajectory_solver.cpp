#include "trajectory_solver.h"
#include "collision_checker.h"
#include "ik_solver.h"
#include "robot_model.h"
#include "utils.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rtfg {

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

  auto t_start = std::chrono::high_resolution_clock::now();

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
  SolverTiming& timing = out.timing;
  timing.n_poses_solved = n;

  auto t_ik_start = std::chrono::high_resolution_clock::now();
  double ik_total_us = 0.0;

  for (int i = 0; i < n; ++i) {
    auto t_pose_start = std::chrono::high_resolution_clock::now();

    std::vector<Eigen::VectorXd> seeds = buildSeedList(q_prev, home_q, robot);
    CandidateInfo best_safe;
    CandidateInfo best_fallback;

    auto run_seed_pool = [&](const std::vector<Eigen::VectorXd>& seed_pool) {
      for (const auto& schedule : weight_schedule) {
        for (const auto& seed : seed_pool) {
          CandidateInfo cand = solveSinglePose(robot, basin_boxes, cfg, target_tforms[i], seed,
                                               q_prev, dq_prev, schedule.first, schedule.second);
          if (!best_fallback.valid || cand.pos_err < best_fallback.pos_err ||
              (std::abs(cand.pos_err - best_fallback.pos_err) < 1e-12 &&
               cand.cost < best_fallback.cost)) {
            best_fallback = cand;
          }
          if (cand.pos_err <= cfg.ik_position_tolerance &&
              cand.rot_err <= schedule.second &&
              cand.clearance >= cfg.clearance_threshold) {
            if (!best_safe.valid || cand.cost < best_safe.cost ||
                (std::abs(cand.cost - best_safe.cost) < 1e-12 &&
                 cand.clearance > best_safe.clearance)) {
              best_safe = cand;
            }
          }
        }
        if (best_safe.valid) break;
      }
    };

    run_seed_pool(seeds);
    if (!best_safe.valid) {
      run_seed_pool(buildGlobalSeedList(robot));
      timing.n_custom_success++;
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
      timing.n_failed++;
      throw std::runtime_error(oss.str());
    }

    auto t_pose_end = std::chrono::high_resolution_clock::now();
    double pose_us =
        std::chrono::duration<double, std::micro>(t_pose_end - t_pose_start).count();
    ik_total_us += pose_us;
    timing.min_per_pose_s = std::min(timing.min_per_pose_s, pose_us * 1e-6);
    timing.max_per_pose_s = std::max(timing.max_per_pose_s, pose_us * 1e-6);

    anchor_q.row(i) = best_safe.q.transpose();
    if (i > 0) {
      out.max_anchor_qstep_deg =
          std::max(out.max_anchor_qstep_deg,
                   rad2deg((wrapJointDelta(best_safe.q - q_prev)).norm()));
    }
    dq_prev = wrapJointDelta(best_safe.q - q_prev);
    q_prev = best_safe.q;

    if (i == 0 || i == n - 1 || ((i + 1) % 10) == 0) {
      std::ostringstream oss;
      oss << "MEX 正在求解轨迹锚点 " << (i + 1) << "/" << n;
      out.progress_events.push_back(
          {"anchor 轨迹求解", 0.05 + 0.45 * ((i + 1.0) / n), oss.str()});
    }
  }

  timing.ik_total_s = ik_total_us * 1e-6;
  timing.avg_per_pose_s = (ik_total_us * 1e-6) / n;
  out.anchor_q = anchor_q;

  // --- playback generation ---
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
    double pos_step = (Tb.block<3, 1>(0, 3) - Ta.block<3, 1>(0, 3)).norm();
    double rot_step = rotationDistance(Ta.block<3, 3>(0, 0), Tb.block<3, 3>(0, 0));
    int nseg = 1 + std::max(
        {2, static_cast<int>(std::ceil(joint_step / (0.70 * M_PI / 180.0))),
         static_cast<int>(std::ceil(pos_step / 0.0030)),
         static_cast<int>(std::ceil(rot_step / (0.30 * M_PI / 180.0)))});
    nseg = std::min(std::max(nseg, 4), 32);
    for (int k = 1; k <= nseg; ++k) {
      double a = quinticBlend(static_cast<double>(k) / static_cast<double>(nseg));
      Eigen::VectorXd q = qa + a * wrapJointDelta(qb - qa);
      playback_q_list.push_back(q);
      playback_segment_names.push_back(
          segment_names.empty() ? std::string() : segment_names[i]);
    }
    if (i == n - 1 || (i % 15) == 0) {
      std::ostringstream oss;
      oss << "MEX 正在生成 playback 轨迹 " << i << "/" << (n - 1);
      out.progress_events.push_back(
          {"playback 连续化", 0.55 + 0.20 * (static_cast<double>(i) / std::max(1, n - 1)),
           oss.str()});
    }
  }

  out.playback_q.resize(static_cast<int>(playback_q_list.size()), 6);
  for (int i = 0; i < static_cast<int>(playback_q_list.size()); ++i) {
    out.playback_q.row(i) = playback_q_list[i].transpose();
    if (i > 0) {
      out.max_playback_qstep_deg =
          std::max(out.max_playback_qstep_deg,
                   rad2deg((wrapJointDelta(playback_q_list[i] - playback_q_list[i - 1])).norm()));
    }
  }
  out.playback_segment_names = playback_segment_names;

  // --- collision audit ---
  auto t_collision_start = std::chrono::high_resolution_clock::now();

  out.tcp_path.resize(out.playback_q.rows(), 3);
  std::vector<Vec3> collision_points;
  for (int i = 0; i < out.playback_q.rows(); ++i) {
    Mat4 T = tipTransform(robot, out.playback_q.row(i).transpose());
    out.tcp_path.row(i) = T.block<3, 1>(0, 3).transpose();
    if (i > 0) {
      out.max_actual_rot_deg = std::max(
          out.max_actual_rot_deg,
          rad2deg(rotationDistance(
              tipTransform(robot, out.playback_q.row(i - 1).transpose()).block<3, 3>(0, 0),
              T.block<3, 3>(0, 0))));
    }
    CollisionSummary cm =
        evaluateConfiguration(robot, basin_boxes, cfg, out.playback_q.row(i).transpose());
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
      collision_points.push_back(T.block<3, 1>(0, 3));
      if (out.first_collision_sample_idx < 0) {
        out.first_collision_sample_idx = i + 1;
        out.first_collision_traj_idx = std::min(i + 1, n);
        out.first_collision_type = cm.violation_type;
        out.first_collision_object = cm.violation_object;
        out.first_collision_segment = playback_segment_names[i];
        out.first_collision_point = T.block<3, 1>(0, 3);
        out.first_collision_clearance = cm.violation_clearance;
      }
    }
    if (i == out.playback_q.rows() - 1 || ((i + 1) % 50) == 0) {
      std::ostringstream oss;
      oss << "MEX 正在进行碰撞复检 " << (i + 1) << "/" << out.playback_q.rows();
      out.progress_events.push_back(
          {"碰撞复检", 0.75 + 0.20 * ((i + 1.0) / out.playback_q.rows()), oss.str()});
    }
  }
  out.collision_points.resize(static_cast<int>(collision_points.size()), 3);
  for (int i = 0; i < static_cast<int>(collision_points.size()); ++i) {
    out.collision_points.row(i) = collision_points[i].transpose();
  }
  for (int i = 1; i < n; ++i) {
    out.max_target_rot_deg = std::max(
        out.max_target_rot_deg,
        rad2deg(rotationDistance(target_tforms[i - 1].block<3, 3>(0, 0),
                                  target_tforms[i].block<3, 3>(0, 0))));
  }

  auto t_collision_end = std::chrono::high_resolution_clock::now();
  timing.collision_total_s =
      std::chrono::duration<double>(t_collision_end - t_collision_start).count();

  auto t_end = std::chrono::high_resolution_clock::now();
  timing.total_wall_s = std::chrono::duration<double>(t_end - t_start).count();

  out.progress_events.push_back({"结果回传", 1.0, "MEX 求解完成"});
  return out;
}

// --- MATLAB struct builders ---

mxArray* buildCollisionResultsStruct(const TrajectoryResult& result) {
  const char* fields[] = {"hasCollision",    "collisionPoints",    "collisionTypes",
                          "collisionObjects", "segmentNames",       "sampleIndices",
                          "trajectoryIndices", "hitBasinNames",     "firstCollision",
                          "minSelfClearance",  "minToolBodyClearance", "minToolBasinClearance",
                          "minSelfObject",     "minToolBodyObject",   "minToolBasinObject",
                          "minSelfTrajectoryIndex", "minToolBodyTrajectoryIndex",
                          "minToolBasinTrajectoryIndex", "minSelfSegmentName",
                          "minToolBodySegmentName", "minToolBasinSegmentName"};
  int nfields = sizeof(fields) / sizeof(fields[0]);
  mxArray* out = mxCreateStructMatrix(1, 1, nfields, fields);

  mxSetField(out, 0, "hasCollision", mxCreateLogicalScalar(result.has_collision));
  mxSetField(out, 0, "collisionPoints", eigenMatrixToMx(result.collision_points));
  mxSetField(out, 0, "collisionTypes", makeStringCellArray(result.collision_types));
  mxSetField(out, 0, "collisionObjects", makeStringCellArray(result.collision_objects));
  mxSetField(out, 0, "segmentNames", makeStringCellArray(result.collision_segments));

  mxArray* sampleIdx =
      mxCreateDoubleMatrix(result.collision_sample_indices.size(), 1, mxREAL);
  mxArray* trajIdx =
      mxCreateDoubleMatrix(result.collision_traj_indices.size(), 1, mxREAL);
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

  const char* fcFields[] = {"sampleIndex", "trajectoryIndex", "segmentName",
                            "type",        "objectName",      "point",
                            "clearance"};
  mxArray* fc = mxCreateStructMatrix(result.first_collision_sample_idx >= 0 ? 1 : 0, 1,
                                     sizeof(fcFields) / sizeof(fcFields[0]), fcFields);
  if (result.first_collision_sample_idx >= 0) {
    mxSetField(fc, 0, "sampleIndex",
               mxCreateDoubleScalar(result.first_collision_sample_idx));
    mxSetField(fc, 0, "trajectoryIndex",
               mxCreateDoubleScalar(result.first_collision_traj_idx));
    mxSetField(fc, 0, "segmentName",
               mxCreateString(result.first_collision_segment.c_str()));
    mxSetField(fc, 0, "type", mxCreateString(result.first_collision_type.c_str()));
    mxSetField(fc, 0, "objectName",
               mxCreateString(result.first_collision_object.c_str()));
    Eigen::Matrix<double, 1, 3> pt = result.first_collision_point.transpose();
    mxSetField(fc, 0, "point", eigenMatrixToMx(pt));
    mxSetField(fc, 0, "clearance", mxCreateDoubleScalar(result.first_collision_clearance));
  }
  mxSetField(out, 0, "firstCollision", fc);

  mxSetField(out, 0, "minSelfClearance",
             mxCreateDoubleScalar(result.global_minimums.min_self));
  mxSetField(out, 0, "minToolBodyClearance",
             mxCreateDoubleScalar(result.global_minimums.min_tool_body));
  mxSetField(out, 0, "minToolBasinClearance",
             mxCreateDoubleScalar(result.global_minimums.min_tool_basin));
  mxSetField(out, 0, "minSelfObject",
             mxCreateString(result.global_minimums.min_self_object.c_str()));
  mxSetField(out, 0, "minToolBodyObject",
             mxCreateString(result.global_minimums.min_tool_body_object.c_str()));
  mxSetField(out, 0, "minToolBasinObject",
             mxCreateString(result.global_minimums.min_tool_basin_object.c_str()));
  mxSetField(out, 0, "minSelfTrajectoryIndex",
             mxCreateDoubleScalar(std::numeric_limits<double>::quiet_NaN()));
  mxSetField(out, 0, "minToolBodyTrajectoryIndex",
             mxCreateDoubleScalar(std::numeric_limits<double>::quiet_NaN()));
  mxSetField(out, 0, "minToolBasinTrajectoryIndex",
             mxCreateDoubleScalar(std::numeric_limits<double>::quiet_NaN()));
  mxSetField(out, 0, "minSelfSegmentName", mxCreateString(""));
  mxSetField(out, 0, "minToolBodySegmentName", mxCreateString(""));
  mxSetField(out, 0, "minToolBasinSegmentName", mxCreateString(""));
  return out;
}

mxArray* buildMetricsStruct(const TrajectoryResult& result) {
  const char* fields[] = {"anchorCount",
                          "playbackCount",
                          "maxTargetRotationDeltaDeg",
                          "maxActualRotationDeltaDeg",
                          "maxAnchorJointStepDegNorm",
                          "maxPlaybackJointStepDegNorm",
                          "moveToTrackTcpDeltaDeg",
                          "moveToTrackJointDeltaDegNorm",
                          "anchorSampleIndices"};
  mxArray* out =
      mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
  mxSetField(out, 0, "anchorCount", mxCreateDoubleScalar(result.anchor_q.rows()));
  mxSetField(out, 0, "playbackCount", mxCreateDoubleScalar(result.playback_q.rows()));
  mxSetField(out, 0, "maxTargetRotationDeltaDeg",
             mxCreateDoubleScalar(result.max_target_rot_deg));
  mxSetField(out, 0, "maxActualRotationDeltaDeg",
             mxCreateDoubleScalar(result.max_actual_rot_deg));
  mxSetField(out, 0, "maxAnchorJointStepDegNorm",
             mxCreateDoubleScalar(result.max_anchor_qstep_deg));
  mxSetField(out, 0, "maxPlaybackJointStepDegNorm",
             mxCreateDoubleScalar(result.max_playback_qstep_deg));
  mxSetField(out, 0, "moveToTrackTcpDeltaDeg", mxCreateDoubleScalar(0.0));
  mxSetField(out, 0, "moveToTrackJointDeltaDegNorm", mxCreateDoubleScalar(0.0));
  mxSetField(out, 0, "anchorSampleIndices", mxCreateDoubleMatrix(0, 0, mxREAL));
  return out;
}

mxArray* buildProgressStruct(const std::vector<ProgressEvent>& events) {
  const char* fields[] = {"phase", "progress", "message"};
  mxArray* out =
      mxCreateStructMatrix(events.size(), 1, sizeof(fields) / sizeof(fields[0]), fields);
  for (mwIndex i = 0; i < events.size(); ++i) {
    mxSetField(out, i, "phase", mxCreateString(events[i].phase.c_str()));
    mxSetField(out, i, "progress", mxCreateDoubleScalar(events[i].progress));
    mxSetField(out, i, "message", mxCreateString(events[i].message.c_str()));
  }
  return out;
}

}  // namespace rtfg
