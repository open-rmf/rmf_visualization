## Changelog for package rmf_visualization

1.2.0 (2021-06-14)
------------------
* Renamed package to `rmf_visualization` with `visualization.launch.xml` launch file: [#8](https://github.com/open-rmf/rmf_visualization/pull/8)
* Renamed visualizer.xml to visualizer.launch.xml: [#6](https://github.com/open-rmf/rmf_visualization/pull/3)
* Removed server.xml: [#7](https://github.com/open-rmf/rmf_visualization/pull/7)


1.1.0 (2020-09-24)
------------------
* Websocket server for negotiation visualizer: [#69](https://github.com/osrf/rmf_schedule_visualizer/issues/69>)
* Improve lift and door marker display for multi-level nav-graphs: [#58](https://github.com/osrf/rmf_schedule_visualizer/issues/58>)
* Contributors: Kevin_Skywalker, Michael X. Grey, Teo Koon Peng, ddengster

1.0.0 (2020-06-22)
------------------
* Provides a single launch file, `visualizer.xml`, to start the `rmf_schedule_visualizer, `fleet_state_visualizer`, `building_systems_visualizer` with RViz as front end.
* Provides a single launch file, `server.xml` to start the `rmf_schedule_visualizer` with a websocket server as back end.
* Contributors: Yadu
