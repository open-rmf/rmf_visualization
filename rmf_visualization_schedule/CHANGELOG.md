## Changelog for package rmf_visualization_schedule

2.0.0 (2022-09-09)
------------------
* Separate map marker and floor plan publishing to separate packages: [#44](https://github.com/open-rmf/rmf_visualization/pull/44)

1.3.0 (2022-03-18)
------------------
* Update to traffic dependency API: [#43](https://github.com/open-rmf/rmf_visualization/pull/43)

1.2.1 (2021-09-01)
------------------
* Fix style checking in CI: [#37](https://github.com/open-rmf/rmf_visualization/pull/37)
* Contributors: Grey

1.2.0 (2021-06-14)
------------------
* Public API for creating of ScheduleDataNode and TrajectoryServer. Single schedule_visualizer executable that publishes schedule markers and starts trajectory websocket server: [#6](https://github.com/open-rmf/rmf_visualization/pull/3)
* Removed dependencies on boost libraries: [#7](https://github.com/open-rmf/rmf_visualization/pull/7)
* Renamed package to `rmf_visualization_schedule`: [#8](https://github.com/open-rmf/rmf_visualization/pull/8)

1.1.0 (2020-09-24)
------------------
* Map markers are only published if a valid Map name is supplied via the SchedulePanel: [#58](https://github.com/osrf/rmf_schedule_visualizer/pull/58)
* Updated dependencies: [#59, #60, #61, #71](https://github.com/osrf/rmf_schedule_visualizer/pull/59)
* Group conflicting participants in websocket server response: [#65](https://github.com/osrf/rmf_schedule_visualizer/pull/65)
* Fix the radius of RViz cylinder markers: [#67](https://github.com/osrf/rmf_schedule_visualizer/pull/67)
* Added capability to publish negotiation status/conclusion messages and negotiation trajectories via websocket: [#69](https://github.com/osrf/rmf_schedule_visualizer/pull/69)
* Convert VisualizerDataNode from reference to shared pointer: [#72](https://github.com/osrf/rmf_schedule_visualizer/pull/72)
* Reduce printouts for websocket server: [#73](https://github.com/osrf/rmf_schedule_visualizer/pull/73)
* Negotiation updates for websocket server are subscribed connections: [#75](https://github.com/osrf/rmf_schedule_visualizer/pull/75)
* Added server_keephistory.xml to launch a server that retains its negotiation history: [#77](https://github.com/osrf/rmf_schedule_visualizer/pull/77)
* Floorplan images are published as OccupancyMaps: [#78](https://github.com/osrf/rmf_schedule_visualizer/pull/78)
* Contributors: Grey, Kevin_Skywalker, Michael X. Grey, Yadu, Yadunund, ddeng, ddengster

1.0.0 (2020-06-22)
------------------
* A ROS 2 node that publishers RViz markers to visualize the itineraries of robots as submitted to the `rmf traffic schedule` along with the expected positions of the robots and their vicinity radii
* Contributors: Aaron, Aaron Chong, Chen Bainian, Grey, Hellyna NG, Marco A. Gutiérrez, Michael X. Grey, Yadu
