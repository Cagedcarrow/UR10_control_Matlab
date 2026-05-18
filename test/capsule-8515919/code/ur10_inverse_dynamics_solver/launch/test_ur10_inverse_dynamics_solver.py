import yaml

from launch import LaunchContext, LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_testing import post_shutdown_test
from launch_testing.actions import ReadyToTest
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc

from ament_index_python.packages import get_package_share_path
import pytest
from unittest import TestCase


@pytest.mark.launch_test
def generate_test_description():
    # Launch context to evaluate parameters
    context = LaunchContext()

    # Dynamics solver plugin name
    inverse_dynamics_solver_plugin_name = ParameterValue(
        "ur10/InverseDynamicsSolverUR10", value_type=str
    ).evaluate(context)

    # Bag file name to test
    package_path = get_package_share_path("ur10_inverse_dynamics_solver")
    bag_filename = ParameterValue("validation_trajectory", value_type=str).evaluate(
        context
    )
    bag_filename = package_path / "bagfiles" / (bag_filename + ".db3")

    # Topic name to read validation trajectory message from
    topic_name = ParameterValue("/joint_space_trajectory", value_type=str).evaluate(
        context
    )

    # Input arguments
    parameters = {
        "inverse_dynamics_solver_plugin_name": inverse_dynamics_solver_plugin_name,
        "bag_filename": str(bag_filename),
        "topic_name": topic_name,
    }

    # Payload inertial parameters
    payload_parameters_path = (
        package_path / "config" / "franka_hand_payload_parameters.yaml"
    )
    with open(payload_parameters_path, "r") as f:
        payload_parameters = yaml.safe_load(f)

    # The node to test
    test_ur10_inverse_dynamics_solver_node = Node(
        package="ur10_inverse_dynamics_solver",
        executable="ur10_inverse_dynamics_solver_test",
        name="test_ur10_inverse_dynamics_solver_node",
        parameters=[parameters, payload_parameters],
        output="screen",
    )

    # Launch the processes and execute tests
    return (
        LaunchDescription(
            [
                test_ur10_inverse_dynamics_solver_node,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        {
            "test_ur10_inverse_dynamics_solver_node": test_ur10_inverse_dynamics_solver_node
        },
    )


class TestTerminatingProcessStops(TestCase):
    def test_gtest_run_complete(
        self, proc_info, test_ur10_inverse_dynamics_solver_node
    ):
        proc_info.assertWaitForShutdown(
            process=test_ur10_inverse_dynamics_solver_node, timeout=4000.0
        )


@post_shutdown_test()
class TestOutcome(TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
