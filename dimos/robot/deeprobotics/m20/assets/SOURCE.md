# DeepRobotics M20 MuJoCo Assets

The model, meshes, locomotion policy, and license in this directory were copied from the official DeepRoboticsLab repository:

- Repository: https://github.com/DeepRoboticsLab/sdk_deploy
- Source commit: `80e3d40084c4ed151ba6f88b0d55cf1d480aa45e`
- Source paths: `src/M20_sdk_deploy/M20_description/m20_mjcf/` and `src/M20_sdk_deploy/policy/policy.onnx`
- License: BSD-3-Clause; see `LICENSE.DeepRobotics`

DimOS removes the source model's floor and light so it can be composed with existing navigation scenes, adds the camera names required by the DimOS sensor adapter, names the IMU sensors, moves collision rendering to geometry group 3 so synthetic depth cameras do not scan the robot, and provides a standing `home` keyframe. Physical collision settings, robot geometry, inertial values, joint limits, actuators, meshes, and ONNX policy are otherwise sourced from the commit above.

The upstream project calls this model `M20`. It is a 16-DOF wheel-legged quadruped. The upstream files do not use the product name `M20 Pro`, so this integration deliberately retains the official `M20` designation.
