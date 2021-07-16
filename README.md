# rmf_schedule_visualizer

![](https://github.com/open-rmf/rmf_visualization/workflows/build/badge.svg)
[![codecov](https://codecov.io/gh/open-rmf/rmf_visualization/branch/main/graph/badge.svg)](https://codecov.io/gh/open-rmf/rmf_visualization)

This repository contains several packages that aid with visualizing various entities within RMF via RViz.
* **rmf_visualization_building_systems**: to visualize the locations and states of lifts and doors in the facility.
* **rmf_visualization_fleet_states**: to visualize the location of robots from various fleets.
* **rmf_visualization_schedule**: to visualize the trajectories of participants in the RMF schedule database. This package also contains a websocket server that provides information on trajectories when queried.
* **rmf_visualization_rviz2_plugins**: Rviz plugins for customizing the view of the participant schedules and interacting with building systems.
* **rmf_visualization**: launch file to bring up the above systems

![](docs/media/visualizer.gif)

## Installation
It is recommended to follow the instructions [here](https://github.com/open-rmf/rmf#rmf) to setup an RMF workspace with the packages in this repository along with other dependencies.

## Run

To launch the visualizer
```
ros2 launch rmf_visualization visualization.launch.xml
```

An active `rmf_traffic_schedule` node is prerequisite for the visualizer to initialize. If a schedule node is not running, it can be started with the command
```
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```
>Note: Only one instance of `rmf_traffic_schedule` must be active at any moment.

The various packages publish MarkerArray messages over these topics:
* `/map_markers` visualizes the nav graphs, waypoints and waypoint labels. Topic durability is `Transient Local`.
* `/schedule_markers` visualizes planned trajectory of the robots in the rmf_schedule along with the robot vicinities.
* `/fleet_markers` renders the current pose of robots as purple spheres
*  `/building_systems_markers` visualize the current states of doors and lifts in the facility. Topic durability is `Transient Local`.

![](docs/media/developer_panels.png)

For a given `map_name`, the `rmf_visualization_schedule_data_node` queries for trajectories in the RMF schedule database over a duration that is specified by `start_duration` and `query_duration` parameters.
The expected location and vicinity of a participant are visualized with concentric yellow and blue circles respectively.
The expected trajectory for a participant is a green polyline when conflict-free and is red otherwise.
The `SchedulePanel` in RViz allows users to modify the parameters used to query trajectories in the schedule database.
`Door` and `Lift` panels allow users to interact with these systems respectively.
The `rmf.rviz` file is used to save the configuration of RViz along with default values of parameters used in the different panels.

## Websocket Server for Custom UIs
For developers looking to create custom UIs outside of the ROS2 environment, this repository provides a websocket server to exchange information contained in an active rmf schedule database. This may primarily be used to query for robot trajectories in the schedule along with conflict information if any. The format for various requests and corresponding responses are described below.

The websocket server starts up when `visualization.launch.xml` is launched.

### Sample Client Requests

#### Server Time
To receive the current server time in milliseconds
```
{"request":"time","param":{}}

```
Sample server response
```
{"response":"time","values":[167165000000]}
```

#### Trajectories in RMF Schedule
To receive a list of active trajectories and conflicts if any between `now` and until a `duration`(milliseconds)
```
{"request":"trajectory","param":{"map_name":"L1","duration":60000, "trim":true}}
```

If `trim` is `false`, data of the complete trajectory is forwarded even if there is only partial overlap in the query duration.

Sample server response
```
{
  "response":"trajectory",
  "values":[{
    "robot_name":"tinyRobot_1",
    "fleet_name:"tinyRobot",
    "shape":"circle",
    "dimensions":0.3,
    "id":310,
    "segments":[
      {"t":336857,"v":[0.018886815933120995,0.4996431607843137,0.0],"x":[11.610485884950174,-8.054053944114406,-1.6085801124572754]},
      {"t":338700,"v":[0.01888681593558913,0.4996431608496359,0.0],"x":[11.645306833719362,-7.132879672805034,-1.6085801124572754]}]}],
  "conflicts":[]
}

```
Here `segments` is a list of dictionaries containing parameters of the knots in the piecewise cubic spline trajectory. `x` stores positional data in [x, y,theta] coordinates while `v`, the velocity data in the same coordinates. `t` is the time recorded in milliseconds.
