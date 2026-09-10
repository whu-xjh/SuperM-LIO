# SuperM-LIO

A LiDAR-inertial odometry and mapping package with an optional moving-object filter.

## Dependencies

- Ubuntu 18.04 / 20.04, ROS1 (desktop-full)
- PCL, Eigen, OpenCV, TBB (Thread Building Blocks), C++17
- **livox_ros_driver** — required at build time (`livox_ros_driver/CustomMsg`) and at runtime for
  Livox LiDARs. This repository does **not** vendor the driver:

  ```bash
  # 1. Livox SDK (v1)
  git clone https://github.com/Livox-SDK/Livox-SDK.git
  cd Livox-SDK && cd build && cmake .. && make && sudo make install

  # 2. livox_ros_driver — use branch `master` (v1 API, provides livox_ros_driver/CustomMsg)
  git clone https://github.com/Livox-SDK/livox_ros_driver.git ws_livox/src/livox_ros_driver
  ```

  Place it inside this workspace's `src/` (or symlink it there) so catkin can find the package.

## Build

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

```bash
roslaunch super_lio Livox_mid360.launch     # other launches: hesai_XT16.launch, hesai_XT32_hilti2022.launch, ...
```

Each launch loads the matching yaml from `super_lio/config/`. Key settings:

- `lio/kf/imu_int_frame` — IMU frames accumulated before initialization (default 50)
- `lio/map/save_map` / `save_interval` — PCD map saving (`map/PCD/scans_*.pcd`, merged on exit;
  the folder is wiped at startup)
- `dyn_obj/en` — enable/disable the moving-object filter (yamls without a `dyn_obj` block run
  with the filter off)

## Acknowledgements

- [M-detector](https://github.com/hku-mars/M-detector) (hku-mars)
