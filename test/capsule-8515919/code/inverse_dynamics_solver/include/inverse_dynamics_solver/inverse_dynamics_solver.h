/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   inverse_dynamics_solver.h
 * Author:  Enrico Ferrentino, Vincenzo Petrone
 * Org.:    UNISA
 * Date:    Dec 3, 2019
 *
 * This class is a plugin interface for dynamic solvers, that are
 * solvers providing the dynamic parameters of a given robot, namely
 * inertia matrix, Coriolis vector, gravity vector and friction
 * vector.
 *
 * -------------------------------------------------------------------
 */

#pragma once

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <ros_extensions_utilities/ros_extensions_utils.h>

namespace inverse_dynamics_solver {

class InverseDynamicsSolver {
  public:
    /**
     * @brief The size of the center of mass (COM) vector
     */
    static const unsigned short int COM_SIZE = 3;

    /**
     * @brief The size of the inertia tensor
     */
    static const unsigned short int INERTIA_SIZE = 6;

    /**
     * @brief Virtual destructor
     */
    virtual ~InverseDynamicsSolver() {}

    /**
     * @brief Initializes the plugin object
     *
     * Since pluginlib requires the implementation to have a parameterless
     * constructor, the objects are initialized through this method
     *
     * @param node node to retrieve the payload inertial parameters
     * @param link_names names of links composing the kinematic chain
     */
    virtual void initialize(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& link_names) {
        // This is to suppress warning [-Wunused-parameter]
        (void)(link_names);

        // Get dynamic parameters from node
        double mass = utils::getParameterFromNode(node, "mass", 0.0);
        std::vector<double> center_of_mass = utils::getParameterFromNode(node, "center_of_mass", std::vector<double>(COM_SIZE, 0.0));
        std::vector<double> inertia = utils::getParameterFromNode(node, "inertia_tensor", std::vector<double>(INERTIA_SIZE, 0.0));

        // Validate dynamic parameters
        validateMass_(mass);
        validateCenterOfMass_(center_of_mass);
        validateInertia_(inertia);

        // Store dynamic parameters
        inertial_parameters_[0] = mass;
        std::copy(center_of_mass.begin(), center_of_mass.end(), inertial_parameters_.begin() + 1);
        std::copy(inertia.begin(), inertia.end(), inertial_parameters_.end() - INERTIA_SIZE);
    }

    /**
     * @brief Get all the dynamic parameters at once
     *
     * @param joint_positions joint positions
     * @param joint_velocities joint velocities
     * @return inertia matrix, Coriolis vector, corresponding to C(q,qd)*qd and gravity vector
     */
    virtual std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd> getDynamicParameters(const Eigen::VectorXd& joint_positions,
                                                                                               const Eigen::VectorXd& joint_velocities) const = 0;

    /**
     * @brief Get the inertia matrix
     *
     * @param joint_positions joint positions
     * @return inertia matrix
     */
    virtual Eigen::MatrixXd getInertiaMatrix(const Eigen::VectorXd& joint_positions) const = 0;

    /**
     * @brief Get the Coriolis vector
     *
     * @param joint_positions joint positions
     * @param joint_velocities joint velocities
     * @return coriolis vector, corresponding to C(q,qd)*qd
     */
    virtual Eigen::VectorXd getCoriolisVector(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities) const = 0;

    /**
     * @brief Get the gravity vector
     *
     * @param joint_positions input joint positions
     * @return gravity vector
     */
    virtual Eigen::VectorXd getGravityVector(const Eigen::VectorXd& joint_positions) const = 0;

    /**
     * @brief Get the vector of friction torques
     *
     * @param joint_velocities input joint velocities
     * @return viscous friction vector
     */
    virtual Eigen::VectorXd getFrictionVector(const Eigen::VectorXd& joint_velocities) const = 0;

    /**
     * @brief Get the vector of torques due to Coriolis effects, inertia and gravity
     *
     * @param joint_positions input joint positions
     * @param joint_velocities input joint velocities
     * @param joint_accelerations input joint accelerations
     * @return torques vector
     */
    virtual Eigen::VectorXd getTorques(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities,
                                       const Eigen::VectorXd& joint_accelerations) const {
        return getInertiaMatrix(joint_positions) * joint_accelerations + getCoriolisVector(joint_positions, joint_velocities) +
               getGravityVector(joint_positions);
    }

    /**
     * @brief Get the payload mass
     *
     * @return the payload mass
     */
    double getPayloadMass() const {
        return inertial_parameters_[0];
    }

    /**
     * @brief Get the payload center of mass
     *
     * @return the payload center of mass
     */
    std::array<double, COM_SIZE> getPayloadCenterOfMass() const {
        std::array<double, COM_SIZE> center_of_mass;
        std::copy(inertial_parameters_.begin() + 1, inertial_parameters_.begin() + 1 + COM_SIZE, center_of_mass.begin());
        return center_of_mass;
    }

    /**
     * @brief Get the payload inertia tensor
     *
     * @return the payload inertia tensor
     */
    std::array<double, INERTIA_SIZE> getPayloadInertia() const {
        std::array<double, INERTIA_SIZE> inertia;
        std::copy(inertial_parameters_.end() - INERTIA_SIZE, inertial_parameters_.end(), inertia.begin());
        return inertia;
    }

    /**
     * @brief Set the payload mass
     *
     * @param mass the payload mass
     */
    void setPayloadMass(double mass) {
        validateMass_(mass);
        inertial_parameters_[0] = mass;
    }

    /**
     * @brief Set the payload center of mass
     *
     * @param center_of_mass the payload center of mass
     */
    void setPayloadCenterOfMass(const std::array<double, COM_SIZE>& center_of_mass) {
        std::copy(center_of_mass.begin(), center_of_mass.end(), inertial_parameters_.begin() + 1);
    }

    /**
     * @brief Set the payload inertia
     *
     * @param inertia the payload inertia
     */
    void setPayloadInertia(const std::array<double, INERTIA_SIZE>& inertia) {
        std::copy(inertia.begin(), inertia.end(), inertial_parameters_.end() - INERTIA_SIZE);
    }

  protected:
    /**
     * @brief Number of payload inertial parameters
     */
    static const unsigned short int N_INERTIAL_PARAMETERS = 10;

    /**
     * @brief Payload inertial parameters stored in the following order: [mass, center_of_mass, inertia_tensor].
     *
     * mass is a scalar, representing the payload mass.
     * center_of_mass is a vector of double of size 3, representing the payload center of mass along x,y,z axes respectively.
     * inertia_tensor is a vector of double of size 6, representing the payload symmetric inertia matrix, in the following order: [Ixx, Ixy, Ixz, Iyy,
     * Iyz, Izz] .
     */
    std::array<double, N_INERTIAL_PARAMETERS> inertial_parameters_;

    InverseDynamicsSolver() {}

  private:
    /**
     * @brief Validates the value of the payload mass to be nonnegative
     *
     * @param mass the payload mass
     *
     * @throws std::length_error if \e mass is negative
     */
    static void validateMass_(double mass) {
        if (mass < 0) {
            std::ostringstream e;
            e << "Mass must be positive, got " << mass;
            throw std::length_error(e.str());
        }
    }

    /**
     * @brief Validates the size of the payload center of mass to be 3
     *
     * @param center_of_mass the payload center of mass
     *
     * @throws std::length_error if size of \e center_of_mass is not 3
     */
    static void validateCenterOfMass_(const std::vector<double>& center_of_mass) {
        if (center_of_mass.size() != COM_SIZE) {
            std::ostringstream e;
            e << "The dimension of the center of mass is wrong; expected " << COM_SIZE << ", got " << center_of_mass.size();
            throw std::length_error(e.str());
        }
    }

    /**
     * @brief Validates the size of the payload inertia tensor to be 6
     *
     * @param inertia the payload inertia tensor
     *
     * @throws std::length_error if size of \e inertia is not 6
     */
    static void validateInertia_(const std::vector<double>& inertia) {
        if (inertia.size() != INERTIA_SIZE) {
            std::ostringstream e;
            e << "The dimension of the inertia tensor is wrong; expected " << INERTIA_SIZE << ", got " << inertia.size();
            throw std::length_error(e.str());
        }
    }
};

}  // namespace inverse_dynamics_solver
