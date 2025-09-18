# Repository Guidelines

## Project Structure & Module Organization
- `src/` — ROS 2 Foxy packages (ament_cmake): `control`, `planning`, `perception`, `localization`, `drivers`, `map`, `bot_msg`, etc. Each package uses `CMakeLists.txt`, `package.xml`, `launch/`, `include/<pkg>/`, `src/`.
- `src/test/` — ROS 2 test/demo packages (e.g., `local_record_test`, `rtk_simulator`).
- `scripts/` — helper scripts: `build/` (colcon helpers), `run/` (launch wrappers, `system_launcher.sh`).
- `path/` — CSV path tooling (e.g., `process_trajectory.py`, `generate_boundaries.py`).
- `doc/` — design/ops notes; `map_files/`, `rviz_files/` — assets; `utils/` — misc tools.

## Build, Test, and Development Commands
- Setup (new shell): `source install/setup.bash` after a build.
- Build all: `colcon build --symlink-install`.
- Build one package: `colcon build --packages-select control --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- Quick helpers: `scripts/build/utils.bash` (build local_record), `scripts/build/test.bash` (build test pkg).
- Run system: `scripts/run/system_launcher.sh full` (see `scripts/run/configs/default_system.yaml`).
- Launch example: `ros2 launch control control.launch.py`.

## Coding Style & Naming Conventions
- C++: `.clang-format` (Google-based), 4-space indent, 100 col limit, sorted includes. Run: `clang-format -i <files>`.
- CMake/ament: keep minimal targets per package, update both `CMakeLists.txt` and `package.xml` when adding deps.
- Python (tools/launch): PEP 8, snake_case files; ROS 2 launch files under `launch/*.py`.
- Topics/frames: align with existing ENU frames (`map`, `base_link`, `lidar_link`) and `bot_msg` definitions.

## Testing Guidelines
- Prefer ROS 2 package-level tests under `src/test/*` (separate test packages). Build selectively with `colcon build --packages-select <test_pkg>` and run via `ros2 launch` where provided.
- For C++ unit tests, use `ament_add_gtest` and register with `ament_cmake` so `colcon test` can discover them.
- Name tests clearly by node/feature (e.g., `control_canbus`), keep deterministic and headless when possible.

## Commit & Pull Request Guidelines
- Commits: imperative, concise (<72 chars), group related changes; examples: `fix routing param parsing`, `add curvature feedback`.
- PRs: clear description, scope of packages touched, validation steps (build/launch commands), logs/screenshots for runtime changes, and linked issues.

## Security & Configuration Tips
- Target Ubuntu 20.04 + ROS 2 Foxy. Required deps are noted in `build.bash` and package manifests.
- Do not hardcode machine-specific paths. Put configs in `scripts/run/configs/` and document defaults in PRs.
