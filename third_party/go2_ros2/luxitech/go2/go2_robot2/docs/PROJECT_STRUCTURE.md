# GO2 Project Structure

This workspace keeps third-party dependencies and custom project code separate.

- `src/unitree-go2-ros2`
  - Third-party simulation and robot stack. Prefer minimal local patches here.
- `src/luxi_go2`
  - Reusable in-house platform packages.
- `src/projects`
  - Project-specific bringup, configuration, and experiment packages.
- `maps`
  - Saved maps and exported map assets.
- `bags`
  - Rosbag recordings.
- `docs`
  - Architecture, interfaces, and development notes.
- `scripts`
  - Workspace-level helper scripts.
