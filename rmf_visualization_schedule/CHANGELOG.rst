^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_visualization_schedule
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


2.0.0 (2022-10-03)
------------------
* Require `rmf_traffic` v3: (`#51 <https://github.com/open-rmf/rmf_visualization/pull/5>`_)
* Separate map marker and floor plan publishing to separate packages: (`#44 <https://github.com/open-rmf/rmf_visualization/pull/4>`_)

1.3.0 (2022-03-18)
------------------
* Update to traffic dependency API: (`#43 <https://github.com/open-rmf/rmf_visualization/pull/4>`_)

1.2.1 (2021-09-01)
------------------
* Fix style checking in CI: (`#37 <https://github.com/open-rmf/rmf_visualization/pull/3>`_)
* Contributors: Grey

1.2.0 (2021-06-14)
------------------
* Public API for creating of ScheduleDataNode and TrajectoryServer. Single schedule_visualizer executable that publishes schedule markers and starts trajectory websocket server: (`#6 <https://github.com/open-rmf/rmf_visualization/pull/>`_)
* Removed dependencies on boost libraries: (`#7 <https://github.com/open-rmf/rmf_visualization/pull/>`_)
* Renamed package to `rmf_visualization_schedule`: (`#8 <https://github.com/open-rmf/rmf_visualization/pull/>`_)

1.1.0 (2020-09-24)
------------------
* Map markers are only published if a valid Map name is supplied via the SchedulePanel: (`#58 <https://github.com/osrf/rmf_schedule_visualizer/pull/5>`_)
* Updated dependencies: (`#59, #60, #61, #71 <https://github.com/osrf/rmf_schedule_visualizer/pull/5>`_)
* Group conflicting participants in websocket server response: (`#65 <https://github.com/osrf/rmf_schedule_visualizer/pull/6>`_)
* Fix the radius of RViz cylinder markers: (`#67 <https://github.com/osrf/rmf_schedule_visualizer/pull/6>`_)
* Added capability to publish negotiation status/conclusion messages and negotiation trajectories via websocket: (`#69 <https://github.com/osrf/rmf_schedule_visualizer/pull/6>`_)
* Convert VisualizerDataNode from reference to shared pointer: (`#72 <https://github.com/osrf/rmf_schedule_visualizer/pull/7>`_)
* Reduce printouts for websocket server: (`#73 <https://github.com/osrf/rmf_schedule_visualizer/pull/7>`_)
* Negotiation updates for websocket server are subscribed connections: (`#75 <https://github.com/osrf/rmf_schedule_visualizer/pull/7>`_)
* Added server_keephistory.xml to launch a server that retains its negotiation history: (`#77 <https://github.com/osrf/rmf_schedule_visualizer/pull/7>`_)
* Floorplan images are published as OccupancyMaps: (`#78 <https://github.com/osrf/rmf_schedule_visualizer/pull/7>`_)
* Contributors: Grey, Kevin_Skywalker, Michael X. Grey, Yadu, Yadunund, ddeng, ddengster

1.0.0 (2020-06-22)
------------------
* A ROS 2 node that publishers RViz markers to visualize the itineraries of robots as submitted to the `rmf traffic schedule` along with the expected positions of the robots and their vicinity radii
* Contributors: Aaron, Aaron Chong, Chen Bainian, Grey, Hellyna NG, Marco A. Gutiérrez, Michael X. Grey, Yadu
