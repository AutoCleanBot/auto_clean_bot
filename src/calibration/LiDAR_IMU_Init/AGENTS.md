# Repository Guidelines

## Project Structure & Module Organization
LI-Init is a ROS catkin package.
- `src/` contains the C++ entry point `laserMapping.cpp`, preprocessing logic, and the IMU helpers compiled into the `li_init` node.
- `include/` hosts reusable libraries (`LI_init`, `ikd-Tree`) and exported headers; keep public headers in this tree.
- `config/` stores sensor-specific YAML profiles; use clear sensor names (for example `livox_avia.yaml`) and document edits.
- `launch/` holds ROS launch files; prefer adding new launch variants rather than editing existing ones in place.
- `python_code/` and `matlab_code/` provide analysis tools; drop scripts here instead of `src/`.

Build artifacts land under `build/`, `devel/`, and `result/`; do not commit generated files.

## Build, Test, and Development Commands
From the catkin workspace root:
- `catkin_make -j$(nproc)` builds the package with the Release profile.
- `source devel/setup.bash` exposes `li_init` messages and executables.
- `roslaunch lidar_imu_init livox_avia.launch` runs the full initialization pipeline (swap launch file per sensor).
Use `docker/docker_start.md` if you need a preconfigured environment.

## Coding Style & Naming Conventions
C++ code targets C++14 with ROS and PCL; follow the existing brace-on-same-line style and four-space indentation. Keep ROS topics snake_case and match config keys to ROS parameters. YAML files should mirror the hardware (`config/ouster*.yaml`), and new ROS messages belong in `msg/` with CamelCase names. Run `catkin_make` before pushing to catch compilation issues; add a clang-format file if you introduce major rewrites.

## Testing Guidelines
No automated tests ship today. Validate changes by replaying representative rosbags (`rosbag play your_data.bag`) alongside the node and confirm that `result/Initialization_result.txt` updates. Use `python_code/result_plot.py` to visualize calibration quality when tuning algorithms. Document the bag, config, and observed offsets in your merge request.

## Commit & Pull Request Guidelines
Recent commits use short, descriptive summaries (for example `Add ZXY,YZX Euler angles`). Write imperative titles under ~60 characters, group related changes, and reference issues or PRs with `(#123)` when relevant. Pull requests should describe the sensor setup, impacted config files, test rosbag(s), and include screenshots or plots when they clarify calibration results. Tag reviewers who own affected modules.
