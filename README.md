![](https://github.com/osrf/rmf_schedule_visualizer/workflows/build/badge.svg)

# rmf_schedule_visualizer

A visualizer for robot trajectories in the `rmf schedule database` and live locations of robots, if available. Users may query trajectories in different maps and view the schedule at a future instance.

![](docs/media/visualizer.gif)

## System Requirements

The visualizer is developed and tested on
[Ubuntu 18.04 LTS](http://releases.ubuntu.com/18.04/) with 
[ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/#installationguide). It can also be run on the `dashing` distribution of ROS2.

## Installation 
```
sudo apt-get update 
sudo apt-get install libeigen3-dev libccd-dev libfcl-dev libyaml-cpp-dev ros-eloquent-rviz2 libwebsocketpp-dev libboost-all-dev -y
mkdir -p ~/ws_rmf/src
cd ~/ws_rmf/src
git clone https://github.com/osrf/rmf_core.git
git clone https://github.com/osrf/traffic_editor.git
git clone https://github.com/osrf/rmf_schedule_visualizer.git
cd ~/ws_rmf
source /opt/ros/eloquent/setup.bash
colcon build --packages-up-to rmf_schedule_visualizer fleet_state_visualizer rviz2_plugin
```

## Usage 

An active `rmf_traffic_schedule` node is prerequisite. This can be started with the command
```
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

Launch the visualizer
```
ros2 launch visualizer visualizer.xml
```
MarkerArray messages are published over three topics,
1)`/map_markers` visualizes the nav graphs, waypoints and waypoint labels. Requires `Transient Local` durability
2)`/schedule_markers` visualizes planned trajectory of the robots in the rmf_schedule
3)`fleet_markers` visualizes the current pose of robots as published over `/fleet_states`

## Submitting a Trajectory
If no active trajectories are present in the schedule, a test trajectory can be submitted for visualization.
```
ros2 run rmf_schedule_visualizer submit_trajectory 

Optional Arguements:
-m <map_name> -x <x_posiiton> -y <y_position> -D <delay> -d <duration>```

Example:
ros2 run rmf_schedule_visualizer submit_trajectory -D 10 -d 20

The submits an "L-shaped" trajectory which spans from `std::chrono::steady_clock::now()` till `duration` seconds has passed. This trajectory is active only after `delay` seconds.
```

The visualizer will also render navigation graphs that are published by `building_map_server` in the `building_map_tools` package of `traffic_editor`.

The live locations of robots as published over`/fleet_states` by various `fleet adapters` can be visualized by running
``` ros2 run fleet_state_visualizer fleet_state_visualizer ```

The visualizer node queries for trajectories in the schedule from the current instance in time until a duration of `query_duration`(s) into the future. The trajectories are also filtered based on a`map_name`. The position of the robot (yellow cylinder) and its conflict-free path (green) are then visualized in RViz2. The schedule can be viewed `start_duration` seconds from the current instance on time by adjusting the slider appropraitely. The default values of `map_name`, `query_duration` and `start_duration` are "B1", 600s and 0s respectively. These values can be modified through the cusom `SchedulePanel` panel GUI in Rviz2. 

## Websocket Backend for Custom UIs 
For developers wanting to develop custom UIs outside of the ROS2 environment, this repository provies a bridge-like node. Running this node starts a websocket server which can receive query requests from clients. The server then responds with information of trajectoties in the `rmf schedule database` that match the query requirements. The format of the request and response messages are described below.

To start the websocket node,

```ros2 run rmf_schedule_visualizer schedule_visualizer -n <node_name> -p <port_number>```

The default <port_number> of the websocet server is `8006`. 

### Client Request Format
To receive the current server time 
```
{"request":"time","param":{}}

```

To recive list of active trajectories between start_time and finish_time
```
{"request":"trajectory","param":{"start_time":"49588000000","finish_time":"4958800000000","map_name":"L1"}}
```
Here the values of `map_name`, `start_time` and `fnish_time` are supplied by the client. `start_time` and `finish_time` are in nanoseconds measured from the start of the epoch. It is recommended to sync the client clock with that of the server.

### Server Response Format 
```
{
  "response" : "trajectory"
  "trajectory" : [ {"shape" : "box",
                    "dimensions" : [1.0, 1.0]
                    "segments" : [{ "x" : [0,0,0], "v" : [0,0,0], "t" : 129129940563641} , {....} , ...] },
                   {"shape" : "circle",
                    "dimensions:" : [1.5]
                    "segments" : .... },
                   {...},
                   ...
                 ]
}

```
Here `segments` is a list of dictionaries containing parameters of the knots in the piecewise cubic spline trajectory. `x` stores positional data in [x, y,theta] coordinates while `v`, the velocity data in the same coordinates. `t` is the time as measured from the start of the epoch.
