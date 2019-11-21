# rmf_schedule_visualizer
##Visualizer for trajectories in the rmf schedule database
This repository contains a backaend component that retrieves trajectories in the rmf schedule database and a front end component that renders the active trajectores into a UI. The user is able to apply time and location filters to the visualizer. 

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

### Demo
The rmf_scheduler_visualizer requires an scheduler node running in the background. A scheduler can be started from the `rmf_traffic_ros2` package.

```ros2 run rmf_traffic_ros2 rmf_traffic_schedule```

Run the schedule visualizer node from rmf_schedule_visualizer

```ros2 run rmf_schedule_visualizer schedule_visualizer -n <node_name> -p <port number>```

The default port of the websocet server is `8006`. 

#### Client Request Structure
```
{
  "request" : "trajectory"
  "param" : {"map_name" : "level1", "start_time" = 0, "finish_time" : 10}
}
```

### Server Response Structure 
```
{
  "response" : "trajectory"
  "trajectory" : [ {"shape" : "box",
                    "dimensions" : [1.0, 1.0]
                    "segments" : [{ "x" : [0,0,0], "v" : [0,0,0], "t" : 10} , {....} , ...] },
                   {"shape" : "circle",
                    "dimensions:" : [1.5]
                    "segments" : .... },
                   {...},
                   ...
                 ]
}

Here `segments` is a list of knots that can be used to generate the cubic spline motions of the trajectories.
```
