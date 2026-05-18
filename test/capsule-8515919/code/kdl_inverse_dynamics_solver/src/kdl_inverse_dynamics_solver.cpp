/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   kdl_inverse_dynamics_solver.cpp
 * Author:  Enrico Ferrentino, Vincenzo Petrone
 * Org.:    UNISA
 * Date:    Dec 3, 2019
 *
 * Refer to the header file for a description of this module.
 *
 * -------------------------------------------------------------------
 */

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>

// KDL
#include <kdl_parser/kdl_parser.hpp>

// Inverse Dynamics Solver
#include <kdl_inverse_dynamics_solver/exceptions.h>
#include <kdl_inverse_dynamics_solver/kdl_inverse_dynamics_solver.h>

using namespace inverse_dynamics_solver;

void InverseDynamicsSolverKDL::initialize(const rclcpp::Node::SharedPtr& node, const std::vector<std::string>& link_names) {
    // Configure payload parameters
    this->InverseDynamicsSolver::initialize(node, link_names);

    // Load robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::make_shared<const robot_model_loader::RobotModelLoader>(node);
    const urdf::ModelInterface robot_model = *robot_model_loader->getModel()->getURDF();

    // Get KDL tree
    KDL::Tree kdl_tree;
    bool tree_ok = kdl_parser::treeFromUrdfModel(robot_model, kdl_tree);
    if (!tree_ok)
        throw kdl_inverse_dynamics_solver::InvalidParameterValueException("The KDL parser cannot parse the URDF loaded in 'robot_description'");

    // Validate KDL chain
    std::string chain_start = link_names.front();
    std::string chain_end = link_names.back();
    bool chain_ok = kdl_tree.getChain(chain_start, chain_end, chain_);
    if (!chain_ok)
        throw kdl_inverse_dynamics_solver::InvalidParameterValueException("Could not create a KDL chain between the provided frames");

    // Instantiate the solver
    number_of_joints_ = chain_.getNrOfJoints();
    KDL::Vector gravity(0, 0, -9.81);
    solver_ = std::make_shared<KDL::ChainDynParam>(chain_, gravity);
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
InverseDynamicsSolverKDL::getDynamicParameters(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities) const {
    KDL::JntArray kdl_joint_positions(number_of_joints_);
    KDL::JntArray kdl_joint_velocities(number_of_joints_);
    KDL::JntSpaceInertiaMatrix H(number_of_joints_);
    KDL::JntArray C(number_of_joints_);
    KDL::JntArray g(number_of_joints_);

    kdl_joint_positions.data = joint_positions;
    kdl_joint_velocities.data = joint_velocities;

    solver_->JntToMass(kdl_joint_positions, H);
    solver_->JntToCoriolis(kdl_joint_positions, kdl_joint_velocities, C);
    solver_->JntToGravity(kdl_joint_positions, g);

    return std::make_tuple(H.data, C.data, g.data);
}

Eigen::MatrixXd InverseDynamicsSolverKDL::getInertiaMatrix(const Eigen::VectorXd& joint_positions) const {
    KDL::JntArray kdl_joint_positions(number_of_joints_);
    KDL::JntSpaceInertiaMatrix H(number_of_joints_);

    kdl_joint_positions.data = joint_positions;

    solver_->JntToMass(kdl_joint_positions, H);

    return H.data;
}

Eigen::VectorXd InverseDynamicsSolverKDL::getCoriolisVector(const Eigen::VectorXd& joint_positions, const Eigen::VectorXd& joint_velocities) const {
    KDL::JntArray kdl_joint_positions(number_of_joints_);
    KDL::JntArray kdl_joint_velocities(number_of_joints_);
    KDL::JntArray C(number_of_joints_);

    kdl_joint_positions.data = joint_positions;
    kdl_joint_velocities.data = joint_velocities;

    solver_->JntToCoriolis(kdl_joint_positions, kdl_joint_velocities, C);

    return C.data;
}

Eigen::VectorXd InverseDynamicsSolverKDL::getGravityVector(const Eigen::VectorXd& joint_positions) const {
    KDL::JntArray kdl_joint_positions(number_of_joints_);
    KDL::JntArray g(number_of_joints_);

    kdl_joint_positions.data = joint_positions;

    solver_->JntToGravity(kdl_joint_positions, g);

    return g.data;
}

Eigen::VectorXd InverseDynamicsSolverKDL::getFrictionVector(const Eigen::VectorXd& joint_velocities) const {
    // KDL joint model does not include friction, thus it is not able to compute the torque vector
    // associated with joint frictions. In the future, this function could be implemented by
    // reading the friction coefficients present in the URDF, meaning that the superclass should
    // be refactored to receive a KinematicModel, instead of a JointModelGroup, this way the
    // URDF can be accessed and friction information can be retrieved.
    (void)(joint_velocities);  // This is to suppress the [-Wunused-parameter] warning
    return Eigen::VectorXd::Zero(number_of_joints_);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(inverse_dynamics_solver::InverseDynamicsSolverKDL, inverse_dynamics_solver::InverseDynamicsSolver)
