# maze_solving_neato

# REPLACE THIS FILE WITH YOUR WRITEUP

CompRobo computer vision project - solving mazes with a Neato

## Setup

### Prerequisites

Install [Pixi](https://pixi.prefix.dev/) if you haven't already:

> **Important:** Remove any `source /opt/ros/jazzy/setup.bash` lines from `~/.bashrc` before installing, they'll conflict with Pixi's environment management.

1. Install Pixi:
```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

2. Add shell completion by appending the following line to `~/.bashrc`:

```bash
eval "$(pixi completion --shell bash)"
```


### Clone and build

On any Linux machine:

```bash
git clone --recurse-submodules <your_repo_url>
cd maze_solving_neato
pixi run build
```

If you forgot `--recurse-submodules`:

```bash
git submodule init && git submodule update
pixi run build
```

No dependencies besides Pixi should be required.

## Available tasks

| Command | What it does |
|---------|-------------|
| `pixi run build` | Build all packages with colcon |
| `pixi run ros-clean` | Delete `build/`, `install/`, `log/` |
| `pixi run maze_solver` | Run `maze_runner` (default). Pass a node name to run a different one: `pixi run maze_solver other_node` |
| `pixi run test` | Run tests for `maze_solver` and print results |
| `pixi run gazebo-empty` | Launch an empty Gazebo world with the Neato |
| `pixi run gazebo-gauntlet` | Launch the gauntlet Gazebo world with the Neato |
| `pixi run connect-neato <IP>` | Connect to a physical Neato, e.g. `pixi run connect-neato 192.168.1.42` |
| `pixi run teleop` | Start keyboard teleoperation (`teleop_twist_keyboard`) |
| `pixi run rviz` | Start RViz2 |
| `pixi run rqt` | Start RQt GUI |
| `pixi run ros2 <command>` | Run any ROS2 command, e.g. `pixi run ros2 topic list` |

## FAQ

### How do I add a new Python library?

Use `pixi add` to install a package from conda-forge or robostack:

```bash
pixi add numpy          # conda-forge package
pixi add ros-jazzy-tf2  # robostack ROS package
```

This updates `pixi.toml` and `pixi.lock`. If a package isn't available on conda-forge, use  `pixi add --pypi <package>` to install from PyPI.
### How do I create and run a new node?

1. Create a new Python file in `src/maze_solver/maze_solver/`, e.g. `my_node.py`, with a `main()` function.
2. Register it as an entry point in `src/maze_solver/setup.py`:
   ```python
   entry_points={
       'console_scripts': [
           'maze_runner = maze_solver.maze_runner:main',
           'my_node = maze_solver.my_node:main',  # add this line
       ],
   },
   ```
3. Rebuild: `pixi run build`
4. Run it: `pixi run maze_solver my_node`

   Or directly: `pixi run ros2 run maze_solver my_node`

### How do I create a new package?

From the workspace root:

```bash
cd src
pixi run ros2 pkg create --build-type ament_python --license MIT --node-name <node_name> <package_name>
cd ..
pixi run build
```

Then add a task for it in `pixi.toml` if you want a shortcut:

```toml
[tasks]
my_new_package = { cmd = "ros2 run my_new_package {args:default_node}" }
```

### How do I set up unit tests?

ROS2 packages created with `ros2 pkg create` include a `test/` directory with linter tests pre-configured. To add your own:

1. Create files starting with `test_` in `src/maze_solver/test/`.
2. Write standard pytest tests:
   ```python
   def test_my_thing():
       assert 1 + 1 == 2
   ```
3. Run them: `pixi run test`

For tests that need a running ROS2 node, use `launch_testing` and add a `test_*.py` that sets up a launch description. See the [ROS2 testing docs](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Testing-Main.html) for examples.

### How do I start multiple nodes at once with a launch file?

Create a launch file in `src/maze_solver/launch/`, e.g. `my_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='maze_solver',
            executable='maze_runner',
            name='maze_runner',
        ),
        Node(
            package='some_other_package',
            executable='some_node',
        ),
    ])
```

Register the launch directory in `src/maze_solver/setup.py`:

```python
import os
from glob import glob

data_files=[
    ...
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
],
```

Rebuild, then run:

```bash
pixi run build
pixi run ros2 launch maze_solver my_launch.py
```

### How do I see what topics are being published?

```bash
pixi run ros2 topic list
pixi run ros2 topic echo /topic_name
pixi run ros2 topic hz /topic_name    # check publish rate
```

### How do I connect to a physical Neato?

```bash
pixi run connect-neato <IP_ADDRESS>
```

### How do I update neato_packages?

```bash
git submodule update --remote neato_packages
pixi run build
```