# kdl_inverse_dynamics_solver

## Contents

This is an implementation of [`InverseDynamicsSolver`](../inverse_dynamics_solver/README.md) using the general-purpose KDL dynamics solver based on [pluginlib](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

It uses the [KDL parser](https://github.com/ros/kdl_parser/tree/humble) to read a robot description from a parameter spawned by [xacro](https://github.com/ros/xacro/tree/ros2).
So, in order to use this library, this parameter must be passed via launch files.
Please refer to the [test section](#how-to-test), spefically to the [test launch file](./launch/test_kdl_inverse_dynamics_solver.py), for an example, and to [the official guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) to know how to pass parameters.

## Dependencies

System dependecies of this package can be installed with `rosdep`, e.g. from this directory run:

```bash
rosdep install --from-paths . --rosdistro humble -i
```

However, `ros_testing` must be installed manually:

```bash
sudo apt install ros-humble-ros-testing
```

## How to build

To build this package, run the following instruction from the root of your colcon workspace:

```bash
colcon build --packages-up-to kdl_inverse_dynamics_solver
```

## How to test

This library is tested against a simulated UR10 robot.
The kinematic description is taken from UR's official package, [ur_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).
The semantic description is taken from UR's official package, [ur_moveit_config](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver).
Both these packages are listed as system dependencies, please check the [related section](#dependencies) to know how to install them.

The tests consist in checking that, given a fixed joint position and velocity state, the KDL solver returns the expected values for the dynamic components.

To build and execute the test, run the following:

```bash
colcon test --packages-select kdl_inverse_dynamics_solver
```

To see the results, run the following:

```bash
colcon test-result --all --verbose
```

The expected output should be the following:

```text
build/kdl_inverse_dynamics_solver/Testing/20240510-0742/Test.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/kdl_inverse_dynamics_solver/test_results/kdl_inverse_dynamics_solver/launch_test_kdl_inverse_dynamics_solver.py.xunit.xml: 2 tests, 0 errors, 0 failures, 0 skipped

Summary: 3 tests, 0 errors, 0 failures, 0 skipped
```

### Optional analysis

If you wish to see the INFO messages printed on console during the test, run the following:

```bash
. install/setup.bash
colcon_cd kdl_inverse_dynamics_solver
launch_test launch/test_kdl_inverse_dynamics_solver.py
```

The expected output is the following:

```text
[INFO] [launch]: All log files can be found below
[INFO] [launch]: Default logging verbosity is set to INFO
test_gtest_run_complete (test_kdl_inverse_dynamics_solver.TestTerminatingProcessStops) ... [INFO] [kdl_inverse_dynamics_solver_test-1]: process started with pid [72266]
[INFO] [python3-2]: process started with pid [72268]
[kdl_inverse_dynamics_solver_test-1] [==========] Running 2 tests from 1 test suite.
[kdl_inverse_dynamics_solver_test-1] [----------] Global test environment set-up.
[kdl_inverse_dynamics_solver_test-1] [----------] 2 tests from InverseDynamicsSolverKDLTest
[kdl_inverse_dynamics_solver_test-1] [ RUN      ] InverseDynamicsSolverKDLTest.dummy
...
[kdl_inverse_dynamics_solver_test-1] [       OK ] InverseDynamicsSolverKDLTest.dummy (18 ms)
[kdl_inverse_dynamics_solver_test-1] [ RUN      ] InverseDynamicsSolverKDLTest.TestDynamicParameters
...
[kdl_inverse_dynamics_solver_test-1] [       OK ] InverseDynamicsSolverKDLTest.TestDynamicParameters (6 ms)
[kdl_inverse_dynamics_solver_test-1] [----------] 2 tests from InverseDynamicsSolverKDLTest (24 ms total)
[kdl_inverse_dynamics_solver_test-1]
[kdl_inverse_dynamics_solver_test-1] [----------] Global test environment tear-down
[kdl_inverse_dynamics_solver_test-1] [==========] 2 tests from 1 test suite ran. (24 ms total)
[kdl_inverse_dynamics_solver_test-1] [  PASSED  ] 2 tests.
[INFO] [kdl_inverse_dynamics_solver_test-1]: process has finished cleanly [pid 72266]
ok

----------------------------------------------------------------------
Ran 1 test in 0.144s

OK
[INFO] [python3-2]: sending signal 'SIGINT' to process[python3-2]
[INFO] [python3-2]: process has finished cleanly [pid 72268]
test_exit_codes (test_kdl_inverse_dynamics_solver.TestOutcome) ... ok

----------------------------------------------------------------------
Ran 1 test in 0.000s

OK
```
