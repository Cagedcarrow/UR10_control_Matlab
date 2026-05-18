/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   ur10_inverse_dynamics_solver.h
 * Author:  Vincenzo Petrone, Antonio Annunziata
 * Org.:    UNISA
 * Date:    Jun 23, 2023
 *
 * This is an implementation of the InverseDynamicsSolver interface
 * from inverse_dynamics_solver, using regressive form of the UR10
 * dynamic model estimated on MATLAB.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <inverse_dynamics_solver/inverse_dynamics_solver.h>

namespace ur10 {

// Motor gains
static const double K1 = 13.5841;
static const double K2 = 14.2959;
static const double K3 = 11.3716;
static const double K4 = 11.2408;
static const double K5 = 11.7681;
static const double K6 = 11.7682;

static const unsigned short int NUMBER_OF_JOINTS = 6;
static const unsigned short int NUMBER_OF_PARAMETERS = 19;
typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, 1> Vector6d;
typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, NUMBER_OF_JOINTS> Matrix6d;
typedef Eigen::Matrix<double, NUMBER_OF_PARAMETERS, 1> Vector19d;
typedef Eigen::Matrix<double, NUMBER_OF_JOINTS, NUMBER_OF_PARAMETERS> Matrix6X19d;

class InverseDynamicsSolverUR10 : public inverse_dynamics_solver::InverseDynamicsSolver {
  public:
    InverseDynamicsSolverUR10() {}

    /**
     * @brief Refer to the superclass documentation
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

    /**
     * @brief Refer to the superclass documentation
     */
    Eigen::VectorXd getTorques(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities,
                               const Eigen::VectorXd& joint_accelerations) const;

  private:
    /**
     * @brief Get the payload dynamic coefficients, i.e. linear combinations of payload dynamic parameters
     *
     * @return the vector of payload dynamic coefficients
     */
    Vector19d getDynamicCoefficients_() const;

    /**
     * @brief This method returns the numerical computation of the current friction vector
     *
     * @param joint_velocities A pointer to a 6-by-1 vector which must contain the current
     *                         values of the joint velocities in [rad/s]
     *
     * @return Returns a 6-by-1 vector with joint currents due to friction (expressed in [A])
     */
    Eigen::VectorXd getFrictionCurrents_(const Eigen::VectorXd& joint_velocities) const;

    /**
     * @brief This method returns the matrix K of motor drive gains, such that tau = K*i
     *
     * @return Returns a 6-by-6 matrix. K will be the diagonal matrix
     *         of the motor drive gains (expressed in [Nm/A]).
     */
    Eigen::MatrixXd getDriveGainsMatrix_() const;
};

}  // namespace ur10
