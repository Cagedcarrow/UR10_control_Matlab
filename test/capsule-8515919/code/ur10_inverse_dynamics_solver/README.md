# ur10_inverse_dynamics_solver

## Contents

This dynamics solver for the real UR10 robot is based on an model identified obtained at current level.
The library [InverseDynamicsSolverUR10](./include/ur10_inverse_dynamics_solver/ur10_inverse_dynamics_solver.h) depends on the following files, automatically generated with MATLAB:

* [getCoriolisCurrents](./include/ur10_inverse_dynamics_solver/getCoriolisCurrents.h): given the 6X1 vector of positions and the 6X1 vector of velocities, both in joint space, it computes the 6x1 vector of currents related to Coriolis effects, espressed in `A`;
* [getCurrents](./include/ur10_inverse_dynamics_solver/getCurrents.h): given the 6X1 vector of positions, the 6X1 vector of velocities and the 6X1 vector of accelerations, all in joint space, it computes the 6x1 vector of currents without the friction effect, espressed in `A`.
* [getGravityCurrents](./include/ur10_inverse_dynamics_solver/getGravityCurrents.h): given the 6x1 vector of positions in joint space, it computes the 6x1 vector of currents due to gravity, espressed in `A`;
* [getInertiaCurrents](./include/ur10_inverse_dynamics_solver/getInertiaCurrents.h): given the 6x1 vector of positions in joint space, it computes the 6x6 inertia matrix, espressed in `A*s^2`;

The dynamics solver implements the [inverse_dynamics_solver::InverseDynamicsSolver](../inverse_dynamics_solver/README.md) class.
Since methods that output torques are expected by the parent, this plugin uses the previously discussed current-based methods, along with the motor gains, to implement the parent class' methods.

## How to build

To build this package, run the following from the root of your colcon workspace:

```bash
colcon build --packages-up-to ur10_inverse_dynamics_solver
```

## How to test

The test consists in validating the solver against a trajectory, on which reference joint positions, velocities and accelerations are stored, and ground-truth joint efforts are specified.
The ground truth is computed with MATLAB, by using an estimated model in regressive form.
The test checks that this library computes the same values as the ground truth.

To build and execute the test, run the following:

```bash
colcon test --packages-select ur10_inverse_dynamics_solver
```

To see the results, run the following:

```bash
colcon test-result --all --verbose
```

The expected output is the following:

```text
build/ur10_inverse_dynamics_solver/Testing/20240510-0742/Test.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/ur10_inverse_dynamics_solver/test_results/ur10_inverse_dynamics_solver/launch_test_ur10_inverse_dynamics_solver.py.xunit.xml: 2 tests, 0 errors, 0 failures, 0 skipped

Summary: 3 tests, 0 errors, 0 failures, 0 skipped
```

### Optional analysis

If you wish to see the INFO messages printed on console during the test, run the following:

```bash
. install/setup.bash
colcon_cd ur10_inverse_dynamics_solver
launch_test launch/test_ur10_inverse_dynamics_solver.py
```

The expected output should look like the following:

```text
[INFO] [launch]: All log files can be found below
[INFO] [launch]: Default logging verbosity is set to INFO
test_gtest_run_complete (test_ur10_inverse_dynamics_solver.TestTerminatingProcessStops) ... [INFO] [ur10_inverse_dynamics_solver_test-1]: process started with pid [10618]
[INFO] [python3-2]: process started with pid [10620]
[ur10_inverse_dynamics_solver_test-1] [==========] Running 3 tests from 1 test suite.
[ur10_inverse_dynamics_solver_test-1] [----------] Global test environment set-up.
[ur10_inverse_dynamics_solver_test-1] [----------] 3 tests from InverseDynamicsSolverUR10Test
[ur10_inverse_dynamics_solver_test-1] [ RUN      ] InverseDynamicsSolverUR10Test.dummy
...
[ur10_inverse_dynamics_solver_test-1] [       OK ] InverseDynamicsSolverUR10Test.dummy (6 ms)
[ur10_inverse_dynamics_solver_test-1] [ RUN      ] InverseDynamicsSolverUR10Test.TestDynamicParameters
...
[ur10_inverse_dynamics_solver_test-1] [       OK ] InverseDynamicsSolverUR10Test.TestDynamicParameters (193 ms)
[ur10_inverse_dynamics_solver_test-1] [ RUN      ] InverseDynamicsSolverUR10Test.TestPayloadInertialParameters
[ur10_inverse_dynamics_solver_test-1] [       OK ] InverseDynamicsSolverUR10Test.TestPayloadInertialParameters (0 ms)
[ur10_inverse_dynamics_solver_test-1] [----------] 3 tests from InverseDynamicsSolverUR10Test (199 ms total)
[ur10_inverse_dynamics_solver_test-1]
[ur10_inverse_dynamics_solver_test-1] [----------] Global test environment tear-down
[ur10_inverse_dynamics_solver_test-1] [==========] 3 tests from 1 test suite ran. (199 ms total)
[ur10_inverse_dynamics_solver_test-1] [  PASSED  ] 3 tests.
[ur10_inverse_dynamics_solver_test-1] [INFO] [1715673079.180703860] [test_ur10_inverse_dynamics_solver_node]: Loading ur10/InverseDynamicsSolverUR10
[ur10_inverse_dynamics_solver_test-1] [INFO] [1715673079.180766512] [test_ur10_inverse_dynamics_solver_node]: ur10/InverseDynamicsSolverUR10 loaded.
[ur10_inverse_dynamics_solver_test-1] [INFO] [1715673079.180769861] [test_ur10_inverse_dynamics_solver_node]: Initializing ur10/InverseDynamicsSolverUR10
[ur10_inverse_dynamics_solver_test-1] [INFO] [1715673079.180783022] [test_ur10_inverse_dynamics_solver_node]: ur10/InverseDynamicsSolverUR10 initialized.
[INFO] [ur10_inverse_dynamics_solver_test-1]: process has finished cleanly [pid 10618]
ok

----------------------------------------------------------------------
Ran 1 test in 0.321s

OK
[INFO] [python3-2]: sending signal 'SIGINT' to process[python3-2]
[INFO] [python3-2]: process has finished cleanly [pid 10620]
test_exit_codes (test_ur10_inverse_dynamics_solver.TestOutcome) ... ok

----------------------------------------------------------------------
Ran 1 test in 0.000s

OK
```

### Further details

The reference bag file [validation_trajectory.db3](./test/bagfiles/validation_trajectory.db3) was originally computed for ROS Noetic.
It was converted for ROS2 Humble with the `rosbags-convert` tool.
Given the `<ros1_bag_file>`, the following command has been launched to generate the ROS2 bag file:

```bash
rosbags-convert --src <ros1_bag_file>.bag --dst validation_trajectory/ --src-typestore ros1_noetic --dst-typestore ros2_humble
```

The output is then generated in the `validation_trajectory/` local folder, containing the new bagfile and [metadata.yaml](./test/bagfiles/metadata.yaml).

**Note 1:** If you don't have `rosbags-convert` in your system, you can install it with

```bash
pip3 install rosbags>=0.9.11
```

**Note 2:** It is possible that `rosbags-convert` is not in your Python path by default.
However, you can launch its executable with

```bash
~/.local/bin/rosbags-convert --help
```

For more details about this tool, please see [this guide](https://docs.openvins.com/dev-ros1-to-ros2.html).
