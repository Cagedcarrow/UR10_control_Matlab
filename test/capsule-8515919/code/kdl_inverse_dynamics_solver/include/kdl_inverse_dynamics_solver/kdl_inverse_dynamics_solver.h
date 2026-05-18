/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   kdl_inverse_dynamics_solver.h
 * Author:  Enrico Ferrentino, Vincenzo Petrone
 * Org.:    UNISA
 * Date:    Dec 3, 2019
 *
 * This is an implementation of DynamicsSolver using the
 * general-purpose, robot-agnostic KDL library.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <moveit/macros/class_forward.h>
#include <kdl/chaindynparam.hpp>
#include <inverse_dynamics_solver/inverse_dynamics_solver.h>

namespace inverse_dynamics_solver {

typedef std::shared_ptr<KDL::ChainDynParam> ChainDynParamPtr;

class InverseDynamicsSolverKDL : public inverse_dynamics_solver::InverseDynamicsSolver {
  public:
    InverseDynamicsSolverKDL() {}

    /**
     * @brief Refer to the superclass documentation
     *
     * When using the KDL Dynamics Solver, make sure to initialize the solver with all links
     * in the dynamic chain. Be aware that, if you use the JointModelGroup class to retrieve
     * the link names, the list might not be complete. For the dynamic model, all links are
     * needed, starting from the parent of the first moving joint to the child of the last
     * moving joint.
     *
     * @throw kdl_inverse_dynamics_solver::Exception if any error occurs during initialization
     */
    void initialize(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& link_names);

    /**
     * @brief Refer to the superclass documentation
     */
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd> getDynamicParameters(const Eigen::VectorXd& joint_positions,
                                                                                       const Eigen::VectorXd& joint_velocities) const;

    /**
     * @brief Refer to the superclass documentation
     */
    Eigen::MatrixXd getInertiaMatrix(const Eigen::VectorXd& joint_positions) const;

    /**
     * @brief Refer to the superclass documentation
     */
    Eigen::VectorXd getCoriolisVector(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities) const;

    /**
     * @brief Refer to the superclass documentation
     */
    Eigen::VectorXd getGravityVector(const Eigen::VectorXd& joint_positions) const;

    /**
     * @brief Refer to the superclass documentation
     */
    Eigen::VectorXd getFrictionVector(const Eigen::VectorXd& joint_velocities) const;

  private:
    unsigned int number_of_joints_;
    KDL::Chain chain_;
    ChainDynParamPtr solver_;
};

}  // namespace inverse_dynamics_solver
