/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   ur10_inverse_dynamics_solver.cpp
 * Author:  Vincenzo Petrone, Antonio Annunziata
 * Org.:    UNISA
 * Date:    Jun 23, 2023
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

// Inverse dynamics solver
#include <ur10_inverse_dynamics_solver/getInertiaCurrents.h>
#include <ur10_inverse_dynamics_solver/getCoriolisCurrents.h>
#include <ur10_inverse_dynamics_solver/getGravityCurrents.h>
#include <ur10_inverse_dynamics_solver/getCurrents.h>
#include <ur10_inverse_dynamics_solver/getPayloadInertiaRegressionMatrix.h>
#include <ur10_inverse_dynamics_solver/getPayloadCoriolisRegressionMatrix.h>
#include <ur10_inverse_dynamics_solver/getPayloadGravityRegressionMatrix.h>
#include <ur10_inverse_dynamics_solver/getPayloadRegressionMatrix.h>
#include <ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.h>

using namespace ur10;

void InverseDynamicsSolverUR10::initialize(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& link_names) {
    this->InverseDynamicsSolver::initialize(node, link_names);
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
InverseDynamicsSolverUR10::getDynamicParameters(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities) const {
    return std::make_tuple(getInertiaMatrix(joint_positions), getCoriolisVector(joint_positions, joint_velocities),
                           getGravityVector(joint_positions));
}

Eigen::MatrixXd InverseDynamicsSolverUR10::getInertiaMatrix(const Eigen::VectorXd& joint_positions) const {
    // Get arm inertia currents
    double q[NUMBER_OF_JOINTS];
    double B[NUMBER_OF_JOINTS * NUMBER_OF_JOINTS];
    Vector6d::Map(q) = joint_positions;
    getInertiaCurrents(q, B);

    // Get payload inertia torques
    double BL[NUMBER_OF_JOINTS * NUMBER_OF_JOINTS];
    double inertial_parameters[N_INERTIAL_PARAMETERS] = { inertial_parameters_[1], inertial_parameters_[2], inertial_parameters_[3],
                                                          inertial_parameters_[4], inertial_parameters_[5], inertial_parameters_[6],
                                                          inertial_parameters_[7], inertial_parameters_[8], inertial_parameters_[9],
                                                          inertial_parameters_[0] };
    getPayloadInertiaRegressionMatrix(q, inertial_parameters, BL);

    // Get inertia torques
    return getDriveGainsMatrix_() * Matrix6d(B) + Matrix6d(BL);
}

Eigen::VectorXd InverseDynamicsSolverUR10::getCoriolisVector(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities) const {
    // Get arm Coriolis and centrigual currents
    double q[NUMBER_OF_JOINTS];
    double qd[NUMBER_OF_JOINTS];
    double coriolis_currents[NUMBER_OF_JOINTS];
    Vector6d::Map(q) = joint_positions;
    Vector6d::Map(qd) = joint_velocities;
    getCoriolisCurrents(q, qd, coriolis_currents);

    // Get payload Coriolis and centrigual torques
    double YCL[NUMBER_OF_JOINTS * NUMBER_OF_PARAMETERS];
    getPayloadCoriolisRegressionMatrix(q, qd, YCL);
    Vector6d payload_coriolis_torques = Matrix6X19d(YCL) * getDynamicCoefficients_();

    // Get Coriolis and centrigual torques
    return getDriveGainsMatrix_() * Vector6d(coriolis_currents) + payload_coriolis_torques;
}

Eigen::VectorXd InverseDynamicsSolverUR10::getGravityVector(const Eigen::VectorXd& joint_positions) const {
    // Get arm gravity currents
    double q[NUMBER_OF_JOINTS];
    double gravity_currents[NUMBER_OF_JOINTS];
    Vector6d::Map(q) = joint_positions;
    getGravityCurrents(q, gravity_currents);

    // Get payload gravity torques
    double YGL[NUMBER_OF_JOINTS * NUMBER_OF_PARAMETERS];
    getPayloadGravityRegressionMatrix(q, YGL);
    Vector6d payload_gravity_torques = Matrix6X19d(YGL) * getDynamicCoefficients_();

    // Get gravity torques
    return getDriveGainsMatrix_() * Vector6d(gravity_currents) + payload_gravity_torques;
}

Eigen::VectorXd InverseDynamicsSolverUR10::getFrictionVector(const Eigen::VectorXd& qd) const {
    return getDriveGainsMatrix_() * getFrictionCurrents_(qd);
}

Eigen::VectorXd InverseDynamicsSolverUR10::getTorques(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities,
                                                      const Eigen::VectorXd& joint_accelerations) const {
    // Get arm currents
    double q[NUMBER_OF_JOINTS];
    double qd[NUMBER_OF_JOINTS];
    double qdd[NUMBER_OF_JOINTS];
    double currents[NUMBER_OF_JOINTS];
    Vector6d::Map(q) = joint_positions;
    Vector6d::Map(qd) = joint_velocities;
    Vector6d::Map(qdd) = joint_accelerations;
    getCurrents(q, qd, qdd, currents);

    // Get payload torques
    double YL[NUMBER_OF_JOINTS * NUMBER_OF_PARAMETERS];
    getPayloadRegressionMatrix(q, qd, qdd, YL);
    Vector6d payload_torques = Matrix6X19d(YL) * getDynamicCoefficients_();

    // Get torques
    return getDriveGainsMatrix_() * Vector6d(currents) + getFrictionVector(joint_velocities) + payload_torques;
}

Vector19d InverseDynamicsSolverUR10::getDynamicCoefficients_() const {
    // Get payload inertial parameters
    double mass = getPayloadMass();
    std::array<double, COM_SIZE> center_of_mass = getPayloadCenterOfMass();
    std::array<double, INERTIA_SIZE> inertia = getPayloadInertia();
    std::array<double, COM_SIZE> moment;  // first moment of inertia
    std::transform(center_of_mass.begin(), center_of_mass.end(), moment.begin(), [mass](double com) { return mass * com; });

    // Get payload dynamic parameters
    Vector19d dynamic_parameters;
    dynamic_parameters << mass, moment[0], moment[1], moment[2], inertia[0], moment[1] * center_of_mass[1], moment[2] * center_of_mass[2], inertia[1],
        moment[0] * center_of_mass[1], inertia[2], moment[0] * center_of_mass[2], inertia[3], moment[0] * center_of_mass[0],
        moment[2] * center_of_mass[2], inertia[4], moment[1] * center_of_mass[2], inertia[5], moment[0] * center_of_mass[0],
        moment[1] * center_of_mass[1];
    return dynamic_parameters;
}

Eigen::VectorXd InverseDynamicsSolverUR10::getFrictionCurrents_(const Eigen::VectorXd& qd) const {
    // friction current vector
    Vector6d friction_current_vector;

    // parameters of the sigmoid friction model
    Vector6d f_v;
    Vector6d f_o;
    Vector6d f_c;
    Vector6d alpha;
    Vector6d ni;

    f_v(0) = 1.064043537306522;
    f_v(1) = 0.994406350204483;
    f_v(2) = 0.679578357403764;
    f_v(3) = 0.315921846615047;
    f_v(4) = 0.224394888674796;
    f_v(5) = 0.235839271499783;

    f_o(0) = -1.006569687742683;
    f_o(1) = 0.956270099079205;
    f_o(2) = -0.811994879948593;
    f_o(3) = -0.176720084521011;
    f_o(4) = -0.192362457059083;
    f_o(5) = -0.245334512427042;

    f_c(0) = 2.050627184327219;
    f_c(1) = -2.401744448718348;
    f_c(2) = 1.647842359055070;
    f_c(3) = 0.468777484116134;
    f_c(4) = 0.475975499951034;
    f_c(5) = 0.597950406691004;

    alpha(0) = 7.946667188885291;
    alpha(1) = -59.953551521104440;
    alpha(2) = 19.825139076376780;
    alpha(3) = 134.8982492898554;
    alpha(4) = 331.4420553811227;
    alpha(5) = 459.1933488839184;

    ni(0) = -0.018479435103176;
    ni(1) = -0.001949874917063;
    ni(2) = -0.005295096397544;
    ni(3) = -0.018576852000623;
    ni(4) = -0.011778095526897;
    ni(5) = -0.012953200294568;

    for (long int i = 0; i < qd.size(); ++i)  // The 'long int' type is to suppress the [-Wsign-compare] warning
        friction_current_vector(i) = f_v(i) * qd(i) + f_o(i) + f_c(i) / (1 + exp(-alpha(i) * (qd(i) + ni(i))));

    return friction_current_vector;
}

Eigen::MatrixXd InverseDynamicsSolverUR10::getDriveGainsMatrix_() const {
    std::vector<double> drive_gains_vector{ K1, K2, K3, K4, K5, K6 };
    return Vector6d(drive_gains_vector.data()).asDiagonal();
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ur10::InverseDynamicsSolverUR10, inverse_dynamics_solver::InverseDynamicsSolver)
