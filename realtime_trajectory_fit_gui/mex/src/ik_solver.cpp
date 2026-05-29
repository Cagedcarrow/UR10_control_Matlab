#include "ik_solver.h"
#include "collision_checker.h"
#include "robot_model.h"
#include "utils.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace rtfg {

CandidateInfo solveSinglePose(const RobotModel& robot,
                              const std::vector<BasinBox>& basin_boxes, const SolverConfig& cfg,
                              const Mat4& target, const Eigen::VectorXd& seed,
                              const Eigen::VectorXd& q_prev, const Eigen::VectorXd& dq_prev,
                              const std::array<double, 6>& weights, double orient_limit) {
  Eigen::VectorXd q = clampToLimits(robot, seed);
  Eigen::VectorXd q_ref = q_prev + dq_prev;

  CandidateInfo cand;
  double prev_err_norm = std::numeric_limits<double>::infinity();
  int stagnate_count = 0;

  for (int iter = 0; iter < cfg.max_iterations; ++iter) {
    Mat4 T = tipTransform(robot, q);
    Vec6 err = poseError(T, target);

    Vec6 weighted = err;
    for (int k = 0; k < 6; ++k) {
      weighted(k) *= weights[k];
    }

    double pos_err = err.head<3>().norm();
    double rot_err = err.tail<3>().norm();

    if (pos_err <= cfg.ik_position_tolerance && rot_err <= orient_limit) {
      break;
    }

    Mat6 J = numericJacobian(robot, q);
    for (int r = 0; r < 6; ++r) {
      J.row(r) *= weights[r];
    }

    // Adaptive damping: higher when far from target, lower when close
    double lambda = cfg.lambda;
    if (pos_err > 0.1 || rot_err > 0.1) {
      lambda = std::max(1e-3, 5e-3 * (pos_err / 0.05));
      lambda = std::min(lambda, 0.1);
    } else {
      lambda = 5e-4 + 5e-3 * (pos_err / 0.05);
    }

    Eigen::MatrixXd H = J.transpose() * J + lambda * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd g = J.transpose() * weighted;
    Eigen::VectorXd dq = H.ldlt().solve(g);

    double step_norm = dq.norm();
    if (step_norm > 0.45) {
      dq *= (0.45 / step_norm);
    }

    q = clampToLimits(robot, q + dq);
    q = alignToReference(robot, q, q_ref);

    // Check convergence rate; break early if stagnating
    double err_norm = weighted.norm();
    if (err_norm >= prev_err_norm * 0.99) {
      stagnate_count++;
      if (stagnate_count >= 8) break;
    } else {
      stagnate_count = 0;
    }
    prev_err_norm = err_norm;
  }

  Mat4 T = tipTransform(robot, q);
  Vec6 err = poseError(T, target);
  auto collision = evaluateConfiguration(robot, basin_boxes, cfg, q);
  double clearance =
      std::min({collision.min_self, collision.min_tool_body, collision.min_tool_basin});

  cand.q = q;
  cand.pos_err = err.head<3>().norm();
  cand.rot_err = err.tail<3>().norm();
  cand.clearance = clearance;
  cand.cost = continuityCost(q, q_prev, dq_prev);
  cand.failure_reason =
      !collision.violation_object.empty() ? collision.violation_object : std::string();
  cand.valid = true;
  return cand;
}

std::vector<Eigen::VectorXd> buildSeedList(const Eigen::VectorXd& q_prev,
                                           const Eigen::VectorXd& home_q,
                                           const RobotModel& robot) {
  std::vector<Eigen::VectorXd> seeds;
  seeds.push_back(q_prev);
  seeds.push_back(home_q);
  seeds.push_back(Eigen::VectorXd::Zero(q_prev.size()));

  std::array<double, 3> wraps = {-2.0 * M_PI, 0.0, 2.0 * M_PI};
  std::vector<Eigen::VectorXd> expanded = seeds;
  for (const auto& q0 : seeds) {
    for (double w5 : wraps) {
      for (double w6 : wraps) {
        if (w5 == 0.0 && w6 == 0.0) continue;
        Eigen::VectorXd q = q0;
        if (q.size() >= 5) q(4) += w5;
        if (q.size() >= 6) q(5) += w6;
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
    if (!duplicate) uniq.push_back(q);
  }
  return uniq;
}

std::vector<Eigen::VectorXd> buildGlobalSeedList(const RobotModel& robot) {
  std::vector<Eigen::VectorXd> seeds;
  Eigen::VectorXd qmid(6);
  for (const auto& seg : robot.segments) {
    if (!seg.movable) continue;
    qmid(seg.q_index) = 0.5 * (seg.lower + seg.upper);
  }
  seeds.push_back(qmid);

  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unif(0.0, 1.0);
  for (int s = 0; s < 96; ++s) {
    Eigen::VectorXd q(6);
    for (const auto& seg : robot.segments) {
      if (!seg.movable) continue;
      q(seg.q_index) = seg.lower + (seg.upper - seg.lower) * unif(rng);
    }
    seeds.push_back(q);
  }
  return seeds;
}

}  // namespace rtfg
