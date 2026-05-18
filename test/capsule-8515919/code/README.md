# Configurable inverse dynamics solver

This repository presents a library to solve the inverse dynamics problem for serial manipulators, i.e. it returns torques or dynamic matrices given input joint positions, velocities and accelerations.

The library is implemented in the [`inverse_dynamics_solver` package](./inverse_dynamics_solver/README.md).
The library is inherited by two concrete classes, i.e.

- A KDL-based solver for simulated robots, based on their robot description defined via Unified Robot Description Format (URDF)
    - You can find it in the [`kdl_inverse_dynamics_solver` package](./kdl_inverse_dynamics_solver/README.md)
- A solver for the real UR10 robot
    - You can find it in the [`ur10_inverse_dynamics_solver` package](./ur10_inverse_dynamics_solver/README.md)

Finally, [utility functions](./ros_extensions_utilities/README.md) are defined as support to tests and demos implemented by the aforementioned packages.

## Dependencies

This code requires the installation of Ubuntu 22.04 and [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html).
