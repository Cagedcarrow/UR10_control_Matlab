/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   test_kdl_inverse_dynamics_solver.cpp
 * Author:  Vincenzo Petrone
 * Org.:    UNISA
 * Date:    May 2, 2024
 *
 * This is a test for InverseDynamicsSolverKDL.
 *
 * -------------------------------------------------------------------
 */

// ROS2
#include <rclcpp/rclcpp.hpp>

// PluginLib
#include <pluginlib/class_loader.hpp>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>

// Gtest
#include <gtest/gtest.h>

// Utilities
#include <ros_extensions_utilities/ros_extensions_utils.h>

// Inverse Dynamics Solver
#include <inverse_dynamics_solver/inverse_dynamics_solver.h>

// This class shares parameters and data across all tests
class SharedData {
    typedef pluginlib::ClassLoader<inverse_dynamics_solver::InverseDynamicsSolver> InverseDynamicsSolverLoader;
    friend class InverseDynamicsSolverKDLTest;

    const char* ROBOT_DESCRIPTION_PARAM = "robot_description";

    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    std::string inverse_dynamics_solver_plugin_name_;
    std::string planning_group_name_;
    std::unique_ptr<InverseDynamicsSolverLoader> inverse_dynamics_solver_loader_;

    SharedData(const SharedData&) = delete;  // this is a singleton
    SharedData() {
        initialize();
    }

    void initialize() {
        // Instantiate the node
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = rclcpp::Node::make_shared("kdl_inverse_dynamics_solver_test", node_options);

        // Load robot model
        RCLCPP_INFO(node_->get_logger(), "Loading robot model from '%s.%s'", node_->get_name(), ROBOT_DESCRIPTION_PARAM);
        robot_model_loader_ = std::make_shared<const robot_model_loader::RobotModelLoader>(node_, ROBOT_DESCRIPTION_PARAM);
        robot_model_ = robot_model_loader_->getModel();
        ASSERT_TRUE(bool(robot_model_)) << "Failed to load robot model";

        // Load parameters
        ASSERT_TRUE(node_->get_parameter("inverse_dynamics_solver_plugin_name", inverse_dynamics_solver_plugin_name_));
        ASSERT_TRUE(node_->get_parameter("planning_group_name", planning_group_name_));
        ASSERT_TRUE(robot_model_->hasJointModelGroup(planning_group_name_));

        // Initialize inverse dynamics solver class loader
        inverse_dynamics_solver_loader_ =
            std::make_unique<InverseDynamicsSolverLoader>("inverse_dynamics_solver", "inverse_dynamics_solver::InverseDynamicsSolver");
        ASSERT_TRUE(bool(inverse_dynamics_solver_loader_)) << "Failed to instantiate ClassLoader<InverseDynamicsSolver>";
    }

  public:
    std::shared_ptr<rclcpp::Node> node_;

    pluginlib::UniquePtr<inverse_dynamics_solver::InverseDynamicsSolver> createUniqueInstance(const std::string& name) const {
        return inverse_dynamics_solver_loader_->createUniqueInstance(name);
    }

    static const SharedData& instance() {
        static SharedData instance;
        return instance;
    }
    static void release() {
        SharedData& shared = const_cast<SharedData&>(instance());
        shared.inverse_dynamics_solver_loader_.reset();
    }
};

// This class implements the tests
class InverseDynamicsSolverKDLTest : public ::testing::Test {
  protected:
    void operator=(const SharedData& data) {
        node_ = data.node_;
        inverse_dynamics_solver_plugin_name_ = data.inverse_dynamics_solver_plugin_name_;
        joint_model_group_ = data.robot_model_->getJointModelGroup(data.planning_group_name_);
    }

    void SetUp() override {
        *this = SharedData::instance();

        // Load KDL inverse dynamics solver plugin
        RCLCPP_INFO_STREAM(node_->get_logger(), "Loading " << inverse_dynamics_solver_plugin_name_);
        inverse_dynamics_solver_ = SharedData::instance().createUniqueInstance(inverse_dynamics_solver_plugin_name_);
        ASSERT_TRUE(bool(inverse_dynamics_solver_)) << "Failed to load plugin: " << inverse_dynamics_solver_plugin_name_;
        RCLCPP_INFO_STREAM(node_->get_logger(), inverse_dynamics_solver_plugin_name_ << " loaded.");

        // Initialize inverse dynamics solver
        RCLCPP_INFO_STREAM(node_->get_logger(), "Initializing " << inverse_dynamics_solver_plugin_name_);
        ASSERT_NO_THROW(inverse_dynamics_solver_->initialize(node_, utils::extractLinkNamesFromJointModelGroup(joint_model_group_)));
        RCLCPP_INFO_STREAM(node_->get_logger(), inverse_dynamics_solver_plugin_name_ << " initialized.");
    }

  public:
    rclcpp::Node::SharedPtr node_;
    std::string inverse_dynamics_solver_plugin_name_;
    std::shared_ptr<inverse_dynamics_solver::InverseDynamicsSolver> inverse_dynamics_solver_;
    moveit::core::JointModelGroup* joint_model_group_;
};

// Tests
// Dummy test
/**
 * @brief verifies that the SetUp() method is called and succeeds
 */
TEST_F(InverseDynamicsSolverKDLTest, dummy) {
    ASSERT_EQ(2 + 2, 4);
}

// Test 1
/**
 * @brief verifies that method getDynamicParameters returns the expected values
 */
TEST_F(InverseDynamicsSolverKDLTest, TestDynamicParameters) {
    // Number of joints
    const unsigned int N_JOINTS = joint_model_group_->getVariableCount();

    // Dynamic components references
    Eigen::MatrixXd inertia_ref(N_JOINTS, N_JOINTS);
    Eigen::VectorXd coriolis_ref(N_JOINTS);
    Eigen::VectorXd gravity_ref(N_JOINTS);

    // Joint states the dynamics will be evaluated on
    Eigen::VectorXd joint_positions(N_JOINTS);
    Eigen::VectorXd joint_velocities(N_JOINTS);

    // Initializing joint positions
    joint_positions(0) = 1.2947;
    joint_positions(1) = 0.2911;
    joint_positions(2) = -1.2749;
    joint_positions(3) = -2.3696;
    joint_positions(4) = -2.7176;
    joint_positions(5) = 0.7307;

    // Initializing joint velocities
    joint_velocities(0) = 0.2541;
    joint_velocities(1) = -0.0215;
    joint_velocities(2) = 0.0271;
    joint_velocities(3) = 0.0000;
    joint_velocities(4) = -0.0319;
    joint_velocities(5) = -0.0082;

    // Reference for inertia matrix
    inertia_ref(0, 0) = 7.51431;
    inertia_ref(0, 1) = -0.0508249;
    inertia_ref(0, 2) = -0.399093;
    inertia_ref(0, 3) = -0.0386227;
    inertia_ref(0, 4) = 0.0100237;
    inertia_ref(0, 5) = 0.000017689;
    inertia_ref(1, 0) = inertia_ref(0, 1);
    inertia_ref(1, 1) = 7.74424;
    inertia_ref(1, 2) = 2.57017;
    inertia_ref(1, 3) = 0.0710538;
    inertia_ref(1, 4) = 0.00311612;
    inertia_ref(1, 5) = -0.000186415;
    inertia_ref(2, 0) = inertia_ref(0, 2);
    inertia_ref(2, 1) = inertia_ref(1, 2);
    inertia_ref(2, 2) = 2.02191;
    inertia_ref(2, 3) = 0.141409;
    inertia_ref(2, 4) = 0.0072951;
    inertia_ref(2, 5) = -0.000186415;
    inertia_ref(3, 0) = inertia_ref(0, 3);
    inertia_ref(3, 1) = inertia_ref(1, 3);
    inertia_ref(3, 2) = inertia_ref(2, 3);
    inertia_ref(3, 3) = 0.0389257;
    inertia_ref(3, 4) = 0.00163918;
    inertia_ref(3, 5) = -0.000186415;
    inertia_ref(4, 0) = inertia_ref(0, 4);
    inertia_ref(4, 1) = inertia_ref(1, 4);
    inertia_ref(4, 2) = inertia_ref(2, 4);
    inertia_ref(4, 3) = inertia_ref(3, 4);
    inertia_ref(4, 4) = 0.00682652;
    inertia_ref(4, 5) = 0;
    inertia_ref(5, 0) = inertia_ref(0, 5);
    inertia_ref(5, 1) = inertia_ref(1, 5);
    inertia_ref(5, 2) = inertia_ref(2, 5);
    inertia_ref(5, 3) = inertia_ref(3, 5);
    inertia_ref(5, 4) = inertia_ref(4, 5);
    inertia_ref(5, 5) = 0.000204525;

    // Reference for Coriolis vector
    coriolis_ref(0) = 0.0254163;
    coriolis_ref(1) = -0.0726293;
    coriolis_ref(2) = -0.175396;
    coriolis_ref(3) = -0.0140663;
    coriolis_ref(4) = -0.000831312;
    coriolis_ref(5) = -4.49748e-07;

    // Reference for gravity vector
    gravity_ref(0) = 0;
    gravity_ref(1) = -101.524;
    gravity_ref(2) = -18.3799;
    gravity_ref(3) = 0.454547;
    gravity_ref(4) = 0.0292181;
    gravity_ref(5) = 0;

    // Test the solver
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd> dyn_params =
        inverse_dynamics_solver_->getDynamicParameters(joint_positions, joint_velocities);
    const double ABS_ERROR = 1e-4;
    for (unsigned int i = 0; i < N_JOINTS; i++) {
        for (unsigned int j = 0; j < N_JOINTS; j++) {
            EXPECT_NEAR(std::get<0>(dyn_params)(i, j), inertia_ref(i, j), ABS_ERROR)
                << "Element (" << i << "," << j << ") of inertia matrix is beyond tolerance";
        }
        EXPECT_NEAR(std::get<1>(dyn_params)(i), coriolis_ref(i), ABS_ERROR) << "Element " << i << " of Coriolis vector is beyond tolerance";
        EXPECT_NEAR(std::get<2>(dyn_params)(i), gravity_ref(i), ABS_ERROR) << "Element " << i << " of gravity vector is beyond tolerance";
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    SharedData::release();
    return result;
}
