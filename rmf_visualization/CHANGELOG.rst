^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2023-06-05)
------------------
* Port version bump and changes from main to humble
* Switch to rst changelogs (`#57 <https://github.com/open-rmf/rmf_visualization/issues/57>`_)
* Contributors: Esteban Martinena, Grey, Yadunund

2.0.1 (2022-11-15)
------------------

2.0.0 (2022-10-03)
------------------
* Update launch file to bringup navgraph, floorplan and obstacle visualizers: (`#44 <https://github.com/open-rmf/rmf_visualization/pull/44>`_)

1.2.1 (2021-09-01)
------------------
* Fix style checking in CI: (`#37 <https://github.com/open-rmf/rmf_visualization/pull/3>`_)
* Contributors: Grey

1.2.0 (2021-06-14)
------------------
* Renamed package to `rmf_visualization` with `visualization.launch.xml` launch file: (`#8 <https://github.com/open-rmf/rmf_visualization/pull/8>`_)
* Renamed visualizer.xml to visualizer.launch.xml: (`#6 <https://github.com/open-rmf/rmf_visualization/pull/6>`_)
* Removed server.xml: (`#7 <https://github.com/open-rmf/rmf_visualization/pull/7>`_)


1.1.0 (2020-09-24)
------------------
* Websocket server for negotiation visualizer: (`#69 <https://github.com/osrf/rmf_schedule_visualizer/pull/69>`_)
* Improve lift and door marker display for multi-level nav-graphs: (`#58 <https://github.com/osrf/rmf_schedule_visualizer/pull/58>`_)
* Contributors: Kevin_Skywalker, Michael X. Grey, Teo Koon Peng, ddengster

1.0.0 (2020-06-22)
------------------
* Provides a single launch file, `visualizer.xml`, to start the `rmf_schedule_visualizer, `fleet_state_visualizer`, `building_systems_visualizer` with RViz as front end.
* Provides a single launch file, `server.xml` to start the `rmf_schedule_visualizer` with a websocket server as back end.
* Contributors: Yadu
