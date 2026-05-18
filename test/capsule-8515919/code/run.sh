#!/usr/bin/env bash
set -ex

# Check for system dependencies
source /opt/ros/humble/setup.bash
rosdep check --from-paths . --rosdistro humble -i -y

# Build packages
colcon build --packages-up-to kdl_inverse_dynamics_solver
colcon build --packages-up-to ur10_inverse_dynamics_solver

# Test
colcon test --packages-select kdl_inverse_dynamics_solver
colcon test --packages-select ur10_inverse_dynamics_solver

# See test results
colcon test-result --all --verbose
