|       | Ubuntu 18.04                                                                               | Ubuntu 20.04                                                                           |
|-------|--------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------|
| Build | ![]( https://github.com/osrf/rmf_schedule_visualizer/workflows/build_eloquent/badge.svg ) | ![]( https://github.com/osrf/rmf_schedule_visualizer/workflows/build_foxy/badge.svg ) |
| Style | ![]( https://github.com/osrf/rmf_schedule_visualizer/workflows/style/badge.svg )          |                                                                                        |

# rmf_schedule_visualizer

A visualizer for robot trajectories in the `rmf schedule database`, live locations of robots if available and states of building systems such as doors and lifts. Users may query trajectories in different maps and view the schedule at a future instance.

![](docs/media/visualizer.gif)

## System Requirements

The visualizer is developed and tested on
* [Ubuntu 18.04 LTS](http://releases.ubuntu.com/18.04/) 
* [ROS2 Eloquent](https://index.ros.org/doc/ros2/Installation/#installationguide).

## Installation 
Install RMF dependencies
```
sudo apt update
sudo apt install -y wget
echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main" > /etc/apt/sources.list.d/gazebo-stable.list
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
sudo apt update
sudo apt install python3-shapely python3-yaml python3-requests \
libignition-common3-dev libignition-plugin-dev libboost-system-dev libboost-date-time-dev libboost-regex-dev libboost-random-dev \
g++-8 -y
```

Setup and build workspace
```
mkdir -p ~/ws_rmf/src
cd ~/ws_rmf/src
git clone https://github.com/osrf/rmf_core.git
git clone https://github.com/osrf/traffic_editor.git
git clone https://github.com/osrf/rmf_schedule_visualizer.git
cd ~/ws_rmf
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro eloquent -yr
source /opt/ros/eloquent/setup.bash
CXX=g++-8 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

## Run 

To launch the visualizer
```
ros2 launch visualizer visualizer.xml
```

An active `rmf_traffic_schedule` node is prerequisite for the visualizer to initialize. If a schedule node is not running, it can be started with the command. Note: More than one schedule node must not be running at any given instance. 
```
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

MarkerArray messages are published over these topics,
* `/map_markers` visualizes the nav graphs, waypoints and waypoint labels. Topic durability is `Transient Local`.
* `/schedule_markers` visualizes planned trajectory of the robots in the rmf_schedule along with the robot vicinities. 
* `/fleet_markers` renders the current pose of robots as purple spheres
*  `/building_systems_markers` visualize the current states of doors and lifts in the facility. Topic durability is `Transient Local`.

![](docs/media/developer_panels.png)

The visualizer node queries for trajectories in the schedule from the current instance in time until a duration of `query_duration`(s) into the future. The trajectories are also filtered based on a`map_name`. The predicted position of the robot (yellow cylinder) and its conflict-free path (green) are then visualized in RViz2. The schedule can be viewed `start_duration` seconds from the current instance on time by adjusting the slider appropraitely. The default values of `map_name`, `query_duration` and `start_duration` are "B1", 600s and 0s respectively. These values can be modified through the cusom `SchedulePanel` panel GUI in Rviz2. A `Door` and `Lift` panel which provide users with manual control over these systems are also included.



## Websocket Server for Custom UIs 
For developers looking to create custom UIs outside of the ROS2 environment, this repository provides a websocket server to exchange information contained in an active rmf schedule database. This may primarily be used to query for robot trajectories in the schedule along with conflict information if any. The format for various requests and corresponding responses are described below.

To start the websocket server,

```ros2 launch visualizer server.xml```

The default port_number of the websocet server is `8006`. 

To test negotiation status trajectories, we can setup negotiations to be retained. Add `-history 999` to the following line in `server.xml`:

``` <node pkg="rmf_schedule_visualizer" exec="schedule_visualizer" args="-p $(var port) -history 999"> ```

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
    "robot_name":"magni1",
    "fleet_name:"magni",
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
