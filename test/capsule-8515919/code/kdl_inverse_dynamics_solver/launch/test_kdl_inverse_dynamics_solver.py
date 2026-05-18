from launch import LaunchContext, LaunchDescription
from launch_param_builder import load_xacro
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

    # UR10 kinematic description
    ROBOT_NAME = "ur10"
    package_path = get_package_share_path("ur_description")
    model_path = package_path / "urdf" / "ur.urdf.xacro"
    robot_description = load_xacro(
        model_path, mappings={"name": ROBOT_NAME, "ur_type": ROBOT_NAME}
    )

    # UR10 semantic description
    package_path = get_package_share_path("ur_moveit_config")
    model_path = package_path / "srdf" / "ur.srdf.xacro"
    robot_description_semantic = load_xacro(model_path, mappings={"name": ROBOT_NAME})

    # Planning group name defining the joints the dynamics solver is evaluated on
    planning_group_name = ParameterValue(
        ROBOT_NAME + "_manipulator", value_type=str
    ).evaluate(context)

    # Dynamics solver plugin name
    inverse_dynamics_solver_plugin_name = ParameterValue(
        "inverse_dynamics_solver/InverseDynamicsSolverKDL", value_type=str
    ).evaluate(context)

    # Input arguments
    parameters = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "planning_group_name": planning_group_name,
        "inverse_dynamics_solver_plugin_name": inverse_dynamics_solver_plugin_name,
    }

    # The node to test
    test_kdl_inverse_dynamics_solver_node = Node(
        package="kdl_inverse_dynamics_solver",
        executable="kdl_inverse_dynamics_solver_test",
        name="test_kdl_inverse_dynamics_solver_node",
        parameters=[parameters],
        output="screen",
    )

    # Launch the processes and execute tests
    return (
        LaunchDescription(
            [
                test_kdl_inverse_dynamics_solver_node,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        {
            "test_kdl_inverse_dynamics_solver_node": test_kdl_inverse_dynamics_solver_node
        },
    )


class TestTerminatingProcessStops(TestCase):
    def test_gtest_run_complete(self, proc_info, test_kdl_inverse_dynamics_solver_node):
        proc_info.assertWaitForShutdown(
            process=test_kdl_inverse_dynamics_solver_node, timeout=4000.0
        )


@post_shutdown_test()
class TestOutcome(TestCase):
    def test_exit_codes(self, proc_info):
        assertExitCodes(proc_info)
