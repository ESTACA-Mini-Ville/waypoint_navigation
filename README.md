# waypoint_navigation

Lightweight ROS1 waypoint navigation package providing a simple graph-based
path planner and a Pure Pursuit path follower. The package is structured so
that planning and following can run in separate nodes (or be embedded in
other processes for testing).

## Highlights

- Path planning on a hard-coded waypoint graph (`waypoint_navigation.graph_data`) using NetworkX.
- Planner publishes a `nav_msgs/Path` on `/planned_path` with orientations computed toward the next waypoint.
- Pure Pursuit follower subscribes to `/planned_path` and `/amcl_pose` and publishes velocity commands on `/cmd_vel`.
- Two small nodes are provided: `route_planner` and `pure_pursuit` (installed as scripts under `nodes/`).

## Contents

- `nodes/route_planner` - node that runs `RouteManager` (listens on `/destination`, publishes `/planned_path`).
- `nodes/pure_pursuit` - node that runs `PurePursuitFollower` (follows `/planned_path`, publishes `/cmd_vel`).
- `src/waypoint_navigation/graph_data.py` - the waypoint graph (list of waypoints and links).
- `src/waypoint_navigation/path_planner.py` - `PathPlanner` class: builds graph and publishes planned paths.
- `src/waypoint_navigation/route_manager.py` - `RouteManager`: receives destination id and triggers planning.
- `src/waypoint_navigation/pure_pursuit_follower.py` - `PurePursuitFollower` controller.
- `launch/waypoint_navigation.launch` - launches both nodes.

## Requirements

- ROS 1 (catkin workspace) — tested on ROS distributions that support Python 3 if your ROS setup uses python3 for scripts; the package's shebangs use `python3` in `nodes/`.
- Python packages: `networkx` (used by the planner).
- ROS packages (declared in `package.xml`): `geometry_msgs`, `nav_msgs`, `rospy`, `std_msgs`.

Install `networkx` in your ROS Python environment if it's missing. Example using pip:

```bash
sudo apt install python3-networkx
```

## Install into a catkin workspace

From the root of your catkin workspace (one level above `src`):

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash  # or use setup.zsh depending on your shell
```

Note: the package already calls `catkin_install_python` for the two node scripts so `catkin build` will make them available via `rosrun`/`roslaunch` after sourcing the workspace.

## Run

Start both the planner and follower with the provided launch file:

```bash
source ~/catkin_ws/devel/setup.bash # or use devel.zsh depending on your shell
roslaunch waypoint_navigation waypoint_navigation.launch
```

The nodes expect localization to publish `PoseWithCovarianceStamped` on `/amcl_pose`.

To request the robot drive to a waypoint (by ID) publish an `Int32` on `/destination`. Example:

```bash
# send a single destination request (ID 15)
rostopic pub -1 /destination std_msgs/Int32 "data: 15"
```

## Topics

- Subscribed:
  - `/destination` (std_msgs/Int32) — destination waypoint id (handled by `route_planner`).
  - `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) — current robot pose for both manager and follower.
  - `/planned_path` (nav_msgs/Path) — used by the follower to track the path.

- Published:
  - `/planned_path` (nav_msgs/Path) — by the planner (latch publisher).
  - `/cmd_vel` (geometry_msgs/Twist) — velocity commands from the Pure Pursuit follower.

## API / Classes

The package exposes a few lightweight classes useful for integration or testing:

- `waypoint_navigation.path_planner.PathPlanner`
  - build_graph(): builds a directed NetworkX graph from the waypoint data.
  - plan(start_id, goal_id): computes shortest path and publishes `/planned_path`.

- `waypoint_navigation.pure_pursuit_follower.PurePursuitFollower`
  - start(): subscribes/publishes to ROS topics (requires `rospy.init_node` called).
  - stop(): unregisters subscribers and stops the robot.

- `waypoint_navigation.route_manager.RouteManager`
  - start(): subscribes to `/destination` and `/amcl_pose` and coordinates planning.

Example: programmatic planning without running a node (requires rospy initialized):

```python
from waypoint_navigation.path_planner import PathPlanner

planner = PathPlanner()
planner.plan(start_id=1, goal_id=15)
```

## Developer notes

- The waypoint graph lives in `graph_data.py`. It currently uses a small transformation that negates `x` and `y` from the original data — adjust there if necessary.
- `PathPlanner` uses NetworkX; this keeps path logic simple and easy to unit-test.
- Nodes are minimal wrappers that initialize rospy and start the respective controller classes so those classes can be instantiated in-process for unit tests.

### Extending

- To add new waypoints or links, edit `src/waypoint_navigation/graph_data.py`.
- To change Pure Pursuit tuning, modify the defaults in `PurePursuitFollower.__init__` (lookahead, speeds, gains).

## Tests

This repo doesn't include automated tests yet. For quick manual checks:

1. Run the launch file.
2. Publish a destination `rostopic pub -1 /destination std_msgs/Int32 "data: <id>"`.
3. Observe `/planned_path` and `/cmd_vel` topics.

