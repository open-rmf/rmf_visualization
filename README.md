# rmf_schedule_visualizer

Visualizer for trajectories in the rmf schedule database

This repository allows for the visualization of trajectories in the rmf schedule. It contains a backend component that retrieves trajectories in the rmf schedule database and a frontend component that renders the active trajectores through a UI. The user is able to specify time and location filters to the visualizer. 

### Installation 
```
mkdir -p ws_rmf/src
cd src
git clone git@github.com:osrf/rmf_core.git
git clone git@github.com:osrf/rmf_schedule_visualizer.git
cd ../
source /opt/ros/dashing/setup.bash
colcon build 
```

### Rviz Demo 

This package also allows for trajectories to be visualized in rviz2. 

```
git checkout rviz2
cd ../
source /opt/ros/dashing/setup.bash
colcon build 
```
First launch rviz2 and load the configuration file located in config/
```
rviz2 rmf_schedule_visualizer/config/rmf.rviz
```
Run the rmf_schedule node
```
ros2 run rmf_traffic_ros2 rmf_traffic_schedule
```

Run the node that publishes trajectory markers to rviz2
```
ros2 run rmf_schedule_visualizer rviz2 -r 5
The default rate(hz) is 1 and can be set using -r flag. For rates above 1, there is usually a 1s delay before markers are erased.
```
A test trajectory can be submitted by
```
ros2 run rmf_schedule_visualizer submit_trajectory -D 10
Additional documentation for this is in the Testing Backend section.
```

The rmf_schedule_visualizer node queries for trajectories in the schedule over a window from the current instance in time until a duration of `query_duration`(s) into the future. The query also requires a`map_name`. The position of the robot (yellow cylinder) and its conflict-free path (green) are visualized in rviz. The schedule can be viewed `start_duration` seconds from the current instance.
The default values of `map_name`, `query_duration` and `start_duration` are "level1", 60s and 0s respectively. These properties can be changed by publihsing an RvizParam msg to /rviz_node/param topic.
```
ros2 topic pub /rviz_node/param rmf_schedule_visualizer_msgs/msg/RvizParam "{map_name: "level2", query_duration: 600, start_duration: 10}" --once

```

### Testing Backend 
The backend of the rmf_scheduler_visualizer is a ros2 node that spins a `Mirror Manager` and a `Websocket Server`. This component can be tested independently to check connectivity and verify messaging formats. The visualizer requires an scheduler node running in the background. If not already running, a scheduler can be started from the `rmf_traffic_ros2` package.

```ros2 run rmf_traffic_ros2 rmf_traffic_schedule```

Next run the schedule visualizer node from rmf_schedule_visualizer

```ros2 run rmf_schedule_visualizer schedule_visualizer -n <node_name> -p <port_number>```

The default <port_number> of the websocet server is `8006`. 

The repo also containts an executable to add trajectories into the schedule which may be useful for testing. 

```ros2 run rmf_schedule_visualizer submit_trajectory -m <map_name> -x <x_posiiton> -y <y_position> -D <delay> -d <duration>```

The submits a trajectory which spans from `std::chrono::steady_clock::now()` till the `duration` has passed. The trajectory has an "L" shaped path. The profile of the trajectory is circular. The start_time of the trajectory can be pushed back by `delay` seconds. 

#### Client Request Format
```
{
  "request" : "trajectory"
  "param" : {"map_name" : "level1", "start_time" : 129109940563641, "finish_time" : 338243159033329}
}
```
Here the values of `map_name`, `start_time` and `fnish_time` are supplied by the client. `start_time` and `finish_time` are in nanoseconds measured from the start of the epoch.

The ros2 node sucscribes to a `<node_name>/debug` topic with message type `std_msgs/String`. On receiving a `c` or `t` message, it prints the count or `start_time` of trajectories active in the Mirror Manager. This may be helpful for testing or debugging purposes.

#### Server Response Format 
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
Here `segments` is a list of dictionaries containing parameters of the knots in the piecewise cubic spline trajectory. `x` stores positional data in [x, y,theta] coordinates while `v`, the velocity data in the same coordinates. `t` is the time as measred from the start of the epoch.
