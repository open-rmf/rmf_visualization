^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_schedule_visualizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.1.0 (2020-09-24)
------------------
* Map markers are only published if a valid Map name is supplied via the SchedulePanel. [#58](https://github.com/osrf/rmf_schedule_visualizer/pull/58)
* Updated dependencies. [#59, #60, #61, #71](https://github.com/osrf/rmf_schedule_visualizer/pull/59)
* Group conflicting participants in websocket server response. [#65](https://github.com/osrf/rmf_schedule_visualizer/pull/65)
* Fix the radius of RViz cylinder markers. [#67](https://github.com/osrf/rmf_schedule_visualizer/pull/67)
* Added capability to publish negotiation status/conclusion messages and negotiation trajectories via websocket. [#69](https://github.com/osrf/rmf_schedule_visualizer/pull/69)
* Convert VisualizerDataNode from reference to shared pointer. [#72](https://github.com/osrf/rmf_schedule_visualizer/pull/72)
* Reduce printouts for websocket server. [#73](https://github.com/osrf/rmf_schedule_visualizer/pull/73)
* Negotiation updates for websocket server are subscribed connections. [#75](https://github.com/osrf/rmf_schedule_visualizer/pull/75)
* Added server_keephistory.xml to launch a server that retains its negotiation history. [#77](https://github.com/osrf/rmf_schedule_visualizer/pull/77)
* Floorplan images are published as OccupancyMaps. [#78](https://github.com/osrf/rmf_schedule_visualizer/pull/78)
* Contributors: Grey, Kevin_Skywalker, Michael X. Grey, Yadu, Yadunund, ddeng, ddengster

1.0.0 (2020-06-22)
------------------
* A ROS 2 node that publishers RViz markers to visualize the itineraries of robots as submitted to the `rmf traffic schedule` along with the expected positions of the robots and their vicinity radii
* Contributors: Aaron, Aaron Chong, Chen Bainian, Grey, Hellyna NG, Marco A. Gutiérrez, Michael X. Grey, Yadu
