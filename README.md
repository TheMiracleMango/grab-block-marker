# Grab Block Marker

ROS-Gazebo package for grabbing blocks in the Gazebo simulation using an interactive marker.

# Project Structure

```
src
├── grab_block_marker
│   ├── ...
│   ├── launch
│   │   └── grab_block_marker.launch
│   └── config
│       └── config.yaml
├── project_gazebo
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── ...
│   └── launch
│       └── project.launch
├── marker_model_description
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── models
│       └── marker.xacro
├── block_1_model_description
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── models
│       └── block.xacro
├── block_2_model_description
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── models
│       └── block.xacro
│   ...
└── block_n_model_description
    ├── CMakeLists.txt
    ├── package.xml
    └── models
        └── block.xacro
```

# Set up

- `grab_block_marker` package sets up the interactive marker to be controlled in Rviz.
  - `grab_block_marker_node` runs the interactive marker node.
  - `config.yaml` configure the grab distance from the marker.
  - `grab_block_marker.launch` file expects the following arguments:
    - `marker_name`: string name for the marker.
    - `block_names`: a list of block names to interact with.
- `project_gazebo` gazebo package to run a simulated environment. It requires:
  - `.launch` file contains:
    - `grab_block_marker.launch` set up:
      - `marker_name`: string name for the marker (e.g., "marker").
      - `block_names`: a list of block names to interact with (e.g., "[block_1, block_2]").
    - Gazebo setup:
      - Gazebo world launch.
      - Model descriptions (optionally from `model_description` packages).
      - Spawn models in the simulation with names specified in the `block_names` argument.
      - Spawn marker model in the simulation with name specified in the `marker_name` argument.
    - Rviz setup (to control the interactive marker set in `grab_block_marker`).
  - (Optional) `model_description` packages to spawn in the Gazebo simulation.

This repository serves as an example setup.
