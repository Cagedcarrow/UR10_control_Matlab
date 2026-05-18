/* -------------------------------------------------------------------
 *
 * The banner will be completed upon code publication.
 *
 * Title:   test_ur10_inverse_dynamics_solver.cpp
 * Author:  Vincenzo Petrone
 * Org.:    UNISA
 * Date:    May 10, 2024
 *
 * This is a test for InverseDynamicsSolverUR10.
 *
 * -------------------------------------------------------------------
 */

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <trajectory_msgs/trajectory_msgs/msg/joint_trajectory.hpp>

// PluginLib
#include <pluginlib/class_loader.hpp>

// Gtest
#include <gtest/gtest.h>

// Utilities
#include <ros_extensions_utilities/ros_extensions_utils.h>

// Inverse Dynamics Solver
#include <inverse_dynamics_solver/inverse_dynamics_solver.h>
#include <ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.h>

// This class shares parameters and data across all tests
class SharedData {
    typedef pluginlib::ClassLoader<inverse_dynamics_solver::InverseDynamicsSolver> InverseDynamicsSolverLoader;
    friend class InverseDynamicsSolverUR10Test;

    std::string inverse_dynamics_solver_plugin_name_;
    std::string bag_filename_;
    std::string topic_name_;
    std::unique_ptr<InverseDynamicsSolverLoader> inverse_dynamics_solver_loader_;

    SharedData(const SharedData&) = delete;  // this is a singleton
    SharedData() {
        initialize();
    }

    void initialize() {
        // Instantiate the node
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node_ = rclcpp::Node::make_shared("ur10_inverse_dynamics_solver_test", node_options);

        // Load parameters
        ASSERT_TRUE(node_->get_parameter("inverse_dynamics_solver_plugin_name", inverse_dynamics_solver_plugin_name_));
        ASSERT_TRUE(node_->get_parameter("bag_filename", bag_filename_));
        ASSERT_TRUE(node_->get_parameter("topic_name", topic_name_));

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
class InverseDynamicsSolverUR10Test : public ::testing::Test {
  protected:
    void operator=(const SharedData& data) {
        node_ = data.node_;
        inverse_dynamics_solver_plugin_name_ = data.inverse_dynamics_solver_plugin_name_;
        bag_filename_ = data.bag_filename_;
        topic_name_ = data.topic_name_;
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
        ASSERT_NO_THROW(inverse_dynamics_solver_->initialize(node_, std::vector<std::string>()));
        RCLCPP_INFO_STREAM(node_->get_logger(), inverse_dynamics_solver_plugin_name_ << " initialized.");
    }

  public:
    rclcpp::Node::SharedPtr node_;
    std::string inverse_dynamics_solver_plugin_name_;
    std::string bag_filename_;
    std::string topic_name_;

    std::shared_ptr<inverse_dynamics_solver::InverseDynamicsSolver> inverse_dynamics_solver_;
};

// Tests
// Dummy test
/**
 * @brief verifies that the SetUp() method is called and succeeds
 */
TEST_F(InverseDynamicsSolverUR10Test, dummy) {
    ASSERT_EQ(2 + 2, 4);
}

// Test 1
/**
 * @brief verifies that method getDynamicParameters returns the expected values
 */
TEST_F(InverseDynamicsSolverUR10Test, TestDynamicParameters) {
    // We test the solver without the payload
    EXPECT_NO_THROW(inverse_dynamics_solver_->setPayloadMass(0.0));
    inverse_dynamics_solver_->setPayloadCenterOfMass(std::array<double, inverse_dynamics_solver::InverseDynamicsSolver::COM_SIZE>());
    inverse_dynamics_solver_->setPayloadInertia(std::array<double, inverse_dynamics_solver::InverseDynamicsSolver::INERTIA_SIZE>());

    // Initialize the rosbag reader
    rclcpp::Serialization<trajectory_msgs::msg::JointTrajectory> serialization;
    RCLCPP_INFO(node_->get_logger(), "Opening '%s'", bag_filename_.c_str());
    rosbag2_cpp::Reader reader;
    reader.open(bag_filename_);

    // For each trajectory in the rosbag file, evaluate the dynamics solver
    while (reader.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr message = reader.read_next();
        if (message->topic_name != topic_name_) {
            continue;
        }
        rclcpp::SerializedMessage serialized_message(*message->serialized_data);
        trajectory_msgs::msg::JointTrajectory::SharedPtr trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        serialization.deserialize_message(&serialized_message, trajectory.get());
        std::size_t n_waypoints = trajectory->points.size();
        RCLCPP_INFO(node_->get_logger(), "Trajectory with %ld waypoints read from %s.", n_waypoints, topic_name_.c_str());

        for (std::size_t i = 0; i < n_waypoints; i++) {
            // Get the current waypoint
            const trajectory_msgs::msg::JointTrajectoryPoint point = trajectory->points[i];

            // Get the torques from the dynamic model's regressive form
            ur10::Vector6d positions = Eigen::Map<const ur10::Vector6d>(point.positions.data());
            ur10::Vector6d velocities = Eigen::Map<const ur10::Vector6d>(point.velocities.data());
            ur10::Vector6d accelerations = Eigen::Map<const ur10::Vector6d>(point.accelerations.data());
            ur10::Vector6d torques = inverse_dynamics_solver_->getTorques(positions, velocities, accelerations);

            // Get the torques by explicit computation
            std::tuple<ur10::Matrix6d, ur10::Vector6d, ur10::Vector6d> dynamic_parameters =
                inverse_dynamics_solver_->getDynamicParameters(positions, velocities);
            ur10::Matrix6d inertia = std::get<0>(dynamic_parameters);
            ur10::Vector6d coriolis = std::get<1>(dynamic_parameters);
            ur10::Vector6d gravity = std::get<2>(dynamic_parameters);
            ur10::Vector6d friction = inverse_dynamics_solver_->getFrictionVector(velocities);
            ur10::Vector6d computed_torques = inertia * accelerations + coriolis + gravity + friction;

            // Compare the torques with the ground truth
            const double ABS_ERROR = 1e-6;
            for (std::size_t j = 0; j < ur10::NUMBER_OF_JOINTS; ++j) {
                EXPECT_NEAR(torques(j), point.effort[j], ABS_ERROR) << "Torque for joint " << j + 1 << " is beyond tolerance at sample " << i + 1;
                EXPECT_NEAR(computed_torques(j), point.effort[j], ABS_ERROR)
                    << "Computed torque for joint " << j + 1 << " is beyond tolerance at sample " << i + 1;
            }
        }
    }
}

// Test 2
/**
 * @brief verifies that the payload inertial parameters have been correctly loaded from YAML
 */
TEST_F(InverseDynamicsSolverUR10Test, TestPayloadInertialParameters) {
    // Reference inertial parameters
    double reference_mass = 0.73;
    std::array<double, inverse_dynamics_solver::InverseDynamicsSolver::COM_SIZE> reference_center_of_mass({ 0.0, 0.01, 0.03 });
    std::array<double, inverse_dynamics_solver::InverseDynamicsSolver::INERTIA_SIZE> reference_inertia({ 0.001, 0.0, 0.0, 0.0025, 0.0, 0.0017 });

    // Compare the loaded parameters with the ground truth
    EXPECT_EQ(inverse_dynamics_solver_->getPayloadMass(), reference_mass);
    EXPECT_EQ(inverse_dynamics_solver_->getPayloadCenterOfMass(), reference_center_of_mass);
    EXPECT_EQ(inverse_dynamics_solver_->getPayloadInertia(), reference_inertia);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    int result = RUN_ALL_TESTS();
    SharedData::release();
    return result;
}
