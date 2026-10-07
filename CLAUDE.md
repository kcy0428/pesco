# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A **colcon (ROS 2) workspace** at `/home/chan/D435i` containing the `realsense-ros` wrapper
(v4.58.4) for Intel RealSense cameras. Target hardware here is a **D435i on a Raspberry Pi 5
(Ubuntu 24.04 arm64), ROS 2 Jazzy**. The workspace root holds the standard colcon layout:
`realsense-ros/` (source), `build/`, `install/`, `log/`.

The `realsense-ros/` source tree is an upstream git clone (remote `realsenseai/realsense-ros`);
the workspace root (`D435i/`) is **not** a git repo. Build artifacts (`build/ install/ log/`)
are committed-to-disk but regenerable.

## Hard dependency: librealsense2

`realsense2_camera/CMakeLists.txt:154` does `find_package(realsense2 2.58.0)` and aborts with a
FATAL_ERROR at line 160 if it is missing. The Intel RealSense SDK (librealsense2) is a **system
dependency, not a ROS package** — colcon/rosdep will not supply it. On arm64/Ubuntu 24.04 there is
no reliable apt package, so it is typically **built from source** (`-DBUILD_EXAMPLES`,
`-DFORCE_RSUSB_BACKEND=ON`) and installed to `/usr/local`. A failing `colcon build` on a fresh
machine almost always means librealsense2 (>= 2.58.0) is absent.

## Build / test commands

Always `source /opt/ros/jazzy/setup.bash` first. Run colcon from the workspace root `D435i/`.

```bash
# Full build (lifecycle node off is the default used here)
colcon build --cmake-args -DUSE_LIFECYCLE_NODE=OFF

# Build one package only
colcon build --packages-select realsense2_camera

# After any build, source the overlay before running nodes/tests
source install/setup.bash

# Run all tests for the camera package (gtest + pytest via ament)
colcon test --packages-select realsense2_camera
colcon test-result --all --verbose          # view results

# Run a single pytest directly (colcon cannot pass pytest -m markers through ament)
python3 -m pytest realsense-ros/realsense2_camera/test/<file>.py
python3 -m pytest ... -m rosbag              # run only a marker group
```

Relevant CMake options (`realsense2_camera/CMakeLists.txt`): `USE_LIFECYCLE_NODE`,
`BUILD_TOOLS` (adds `realsense2_frame_latency_node`), `BUILD_ACCELERATE_GPU_WITH_GLSL`,
`BUILD_ACCELERATE_GPU_WITH_CUDA`.

## Running the camera

```bash
ros2 launch realsense2_camera rs_launch.py                     # single camera
ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true  # D435i IMU
ros2 launch realsense2_camera rs_multi_camera_launch.py        # multiple cameras
```

Launch files live in `realsense2_camera/launch/`. End-to-end usage examples (pointcloud,
align_depth, rosbag, dual camera) are in `realsense2_camera/examples/*/`.

## Architecture

Four ament packages under `realsense-ros/`:

- **`realsense2_camera`** — the wrapper node (C++). This is where almost all logic lives.
- **`realsense2_camera_msgs`** — `.msg`/`.srv`/`.action` interfaces (rosidl). Build this first;
  the camera package depends on its generated typesupport.
- **`realsense2_description`** — URDF/xacro and meshes for the camera models.
- **`realsense2_rgbd_plugin`** — RViz2 plugin for the combined RGBD topic.

### Camera node (`realsense2_camera/src/`)

The node is a **composable component**, registered as the ROS plugin
`realsense2_camera::RealSenseNodeFactory` → executable `realsense2_camera_node`
(`CMakeLists.txt:395`). Flow:

- **`realsense_node_factory.cpp`** — the component entry point. Owns the librealsense `rs2::context`,
  discovers/hot-plugs the physical device (`changeDeviceCallback`), handles initial reset, then
  constructs and owns a single `BaseRealSenseNode` (`_realSenseNode`). Device lifecycle (connect /
  disconnect / reset) is managed here, separate from streaming logic.
- **`base_realsense_node.cpp`** — the core. Wires every librealsense sensor into ROS: starts
  streams, pumps frames into publishers, builds and broadcasts TF, exposes services. The largest
  file; most feature work touches it.
- **Filters** (`named_filter.cpp`, `align_depth_filter.cpp`, `pointcloud_filter.cpp`) — wrappers
  over librealsense post-processing blocks, each toggled at runtime via a `<filter>.enable` param.
- **Parameters** (`parameters.cpp`, `sensor_params.cpp`, `dynamic_params.cpp`,
  `ros_param_backend.cpp`) — declare + live-update ROS params. Many stream/filter params are
  reconfigurable at runtime; this layer bridges ROS params to librealsense sensor options.
- **`ros_sensor.cpp` / `profile_manager.cpp`** — map ROS-side stream config
  (e.g. `depth_module.profile:=640x480x30`) onto librealsense sensors and stream profiles.
- **`image_publisher.cpp`** — supports intra-process zero-copy image transport (see the
  `rs_intra_process_demo_launch.py` demo).
- **`tfs.cpp`, `ros_utils.cpp`, `actions.cpp`, `safety.cpp`** — TF math (note ROS vs. optical
  coordinate frames, explained in README), helpers, action servers, and safety-camera (D500) bits.

### Parameter model (important when reading README/params)

Post-ros2-legacy conventions, enforced throughout `parameters.cpp`/`profile_manager.cpp`:
- Video streams are configured with `<module>.profile:=WxHxFPS` (e.g. `depth_module.profile`,
  `rgb_camera.profile`) — **not** per-stream width/height/fps params.
- Every filter and sensor is enabled/disabled via `<name>.enable`; there is no `filters` list.
- `align_depth` and `pointcloud` are filters: `align_depth.enable`, `pointcloud.enable`.

## Tests

`realsense2_camera/test/` mixes **gtest** (unit, files `gtest_*.cpp`) and **pytest** (integration,
files `test_*.py`). CMake auto-globs by filename prefix, so a new test only needs the right name in
a registered folder (folders are listed explicitly in `CMakeLists.txt` under `_gtest_folders` /
`_pytest_folders`). Integration tests subclass `pytest_rs_utils.RsTestBaseClass`
(`test/utils/`) with init / run_test / process_data steps. Many tests need real hardware and use
markers (`d435i`, `d457`, `rosbag`, …) to gate on available devices. See
`realsense2_camera/test/README.md` for the full convention; start new tests from the
`gtest_template.cpp` / `test_integration_template.py` / `test_launch_template.py` templates.
