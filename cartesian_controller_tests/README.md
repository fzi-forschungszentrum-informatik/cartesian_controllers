# Cartesian Controller Tests
Integration tests and high-level concept validation for the *cartesian_controllers*.
Unit tests are located in each package's `test` sub-folder.

Useful information on integration tests in ROS2 is available [here][1]

## Run tests manually
In a sourced terminal, call
```bash
colcon test --packages-select cartesian_controller_tests && colcon test-result --test-result-base build/cartesian_controller_tests/ --verbose
```
to run and inspect the integration tests locally.

[1]: https://github.com/ros2/launch/tree/master/launch_testing#quick-start-example
