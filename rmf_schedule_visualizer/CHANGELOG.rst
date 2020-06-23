^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_schedule_visualizer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-22)
------------------
* Merge pull request `#43 <https://github.com/osrf/rmf_schedule_visualizer/issues/43>`_ from osrf/server/apppend_names
  Server/apppend names
* Fixed merge conflicts
* Merge pull request `#53 <https://github.com/osrf/rmf_schedule_visualizer/issues/53>`_ from osrf/bug/markers
  Bug/markers
* Merge branch 'master' into feature/lift-and-door-panels
* Merge pull request `#51 <https://github.com/osrf/rmf_schedule_visualizer/issues/51>`_ from osrf/no_boxes
  Update to the rmf_core develop API
* Cleanup
* Fixed undefined behavior with compute_cubic_splines
* Update API usage
* Remove Box headers because they are not supported any longer
* Merge pull request `#49 <https://github.com/osrf/rmf_schedule_visualizer/issues/49>`_ from osrf/develop
  Merge develop
* RMF Linter
* Fixed merge conflicts
* Merge pull request `#50 <https://github.com/osrf/rmf_schedule_visualizer/issues/50>`_ from osrf/feature/door_lift_markers
  Feature/door lift markers
* Added lift and door markers
* Merge pull request `#48 <https://github.com/osrf/rmf_schedule_visualizer/issues/48>`_ from Briancbn/pr-fix-rosdep-websocketpp-error
  Fix rosdep websocketpp error
* Merge pull request `#47 <https://github.com/osrf/rmf_schedule_visualizer/issues/47>`_ from osrf/feature/show_vicinity
  Show the vicinity alongside the footprint
* Fix rosdep websocketpp not found error
* Fixed merge conflicts
* Merge pull request `#46 <https://github.com/osrf/rmf_schedule_visualizer/issues/46>`_ from osrf/fix/rmf-traffic-msgs-dep
  fixed missing REQUIRED tag and package dependency
* fixed missing REQUIRED tag and package dependency
* Apppended robot and fleet name to trajectory response
* Merge pull request `#40 <https://github.com/osrf/rmf_schedule_visualizer/issues/40>`_ from osrf/feature/server_conflicts
  Webserver response with schedule conflict data
* Server publishes conflict data
* Cleanup
* Server response includes conflict data
* Added parent info to check_limit
* Show the vicinity alongside the footprint
* Added check for numerical limits
* Disabled location marker
* Fixed build errors
* Fixed merge conflicts
* Merge pull request `#39 <https://github.com/osrf/rmf_schedule_visualizer/issues/39>`_ from osrf/server/trim_trjectories
  Trajectory requests accepts a trim parameter
* Fixed bug when requested finish time exceeds trajectiory finish time
* Fixed trim
* Trajectory requests accepts a trim parameter
* Merge pull request `#37 <https://github.com/osrf/rmf_schedule_visualizer/issues/37>`_ from osrf/fix/server
  Fix/server
* Time parameter in trajectory response is integer representing milliseconds
* Remove debug output
* Cleanup
* Changed the expectation of duration parameter to number
* Trajectory request requires only map_name and duration. Response also includes id
* Update to new conflict system
* Websocket server starts inside main_rviz
* Merge pull request `#33 <https://github.com/osrf/rmf_schedule_visualizer/issues/33>`_ from osrf/feature/waypoint_names
  Feature/waypoint names
* Updated default config
* Format fixes
* Split dp2_marker_array into schedule_markers and map_markers. map_markers has transient local durability.
* Added label markers
* Merge pull request `#31 <https://github.com/osrf/rmf_schedule_visualizer/issues/31>`_ from osrf/feature/name_markers
  Added text marker to marker array
* Added text marker to marker array
* Debugging
* Merge pull request `#30 <https://github.com/osrf/rmf_schedule_visualizer/issues/30>`_ from osrf/fix/rviz_config
  Fix config
* Fixed view
* Updating the API
* Update VisualizerData.cpp
* Merge pull request `#28 <https://github.com/osrf/rmf_schedule_visualizer/issues/28>`_ from osrf/fix_format
  Fix format
* Format fixes
* Formatting fixes
* Reformatting
* Merge pull request `#27 <https://github.com/osrf/rmf_schedule_visualizer/issues/27>`_ from osrf/add_launch
  Add launch
* Cleanup
* Refactor launch
* Added visualizer.xml launch file
* Merge pull request `#26 <https://github.com/osrf/rmf_schedule_visualizer/issues/26>`_ from osrf/server_updates
  Time parameters are exchanged as strings over websocket connection
* added response for client request of current time
* modified format of response
* time parameters are exchanges a strings over websocket
* Merge branch 'master' into frontend-slider-control
* Merge pull request `#21 <https://github.com/osrf/rmf_schedule_visualizer/issues/21>`_ from osrf/config
  Config
* changed map topic
* updated marker sizes
* tweaked marker alphas and scale of fleet state markers
* layout change
* Merge pull request `#18 <https://github.com/osrf/rmf_schedule_visualizer/issues/18>`_ from osrf/fix_map_cache
  Update Level Cache
* level cache updates during initialization. Default Map Name set to B1
* Merge pull request `#17 <https://github.com/osrf/rmf_schedule_visualizer/issues/17>`_ from osrf/fix_merge_duplicates
  Minor Fix
* removed duplicate definition of _marker_array_pub. formatting fixes
* fixed merge conflicts
* Merge pull request `#15 <https://github.com/osrf/rmf_schedule_visualizer/issues/15>`_ from osrf/display_fixes
  Do not wait before displaying trajectories
* added upper bound guard for start_duration update
* fixed location marker visualization
* Do not wait before displaying trajectories
* Merge pull request `#14 <https://github.com/osrf/rmf_schedule_visualizer/issues/14>`_ from osrf/fix_threading
  Fix the number of threads used by MultiThreadedExecutor
* remove debug printout
* remove debug printout
* Fix the number of threads used by MultiThreadedExecutor
* added more colors
* updated make_point()
* smaller node markers in white
* path marker reflects query_duration
* fixed lane marker id
* added function to create colors. map graph id to color
* reduced with of path. changed map marker type to line_list
* visualize nodes
* added debug messages
* reduced width of lane marker and added lifetime
* changed default value of query duration to 600s
* added lane markers
* added cache for level msg
* skeleton for map markers
* updated callback for map sub
* formatting fixes
* added subscriber to building map msgs
* added building_map_msgs dependency
* Visualize the schedule using markers in rviz2 (`#13 <https://github.com/osrf/rmf_schedule_visualizer/issues/13>`_)
* Merge branch 'master' of github.com:osrf/rmf_schedule_visualizer
* cmake formatting fix
* Merge pull request `#8 <https://github.com/osrf/rmf_schedule_visualizer/issues/8>`_ from osrf/enhance_debug
  Enhance debug
* added to info debug
* added 'info' in debug
* Merge pull request `#7 <https://github.com/osrf/rmf_schedule_visualizer/issues/7>`_ from osrf/implement_mutex
  Implement mutex
* updated submit_trajectory service call to submit_trajectories
* added copyright
* added info msg for trajectory start_time callback
* added mutex lock_guards when accessing mirror in VisualizerData.cpp
* fixed problem with exiting submit_trajecotry while waiting for service
* Merge pull request `#6 <https://github.com/osrf/rmf_schedule_visualizer/issues/6>`_ from osrf/submit_multiple_trajectories
  Positional args for submit_trajectory
* added command line arguements for passing positional data for stationary trajectories
* Merge pull request `#5 <https://github.com/osrf/rmf_schedule_visualizer/issues/5>`_ from osrf/backend_add_server
  Backend add server
* fixed typo in key name
* added feature to read radius of circular profile. But needs to be generalized
* added wait for submit_trajectory service to become available
* corrected client service name
* fixed node initialization
* added executable to submit trajectory to scheduler for debugging
* fixed conversion from json field to rmf_traffic::Time
* removed option to set mirror in the server instance after initialization
* added parse_trajectory code
* cleaned up commented code
* added function skeletons to parse client request and generate response
* fixed spacetime query
* reformatted functions
* added header for json parsing
* Server builder function runs the server automatically after initialization
* added port arguement to main
* updated main with instance of server
* modified CMakeLists.txt and package.xml
* added skeleton for websocket server
* added function to parse segments of trajectories in mirror
* added debug subscriber
* skeleton code to spin node with mirror manager
* added source and header files
* initial commit
* Contributors: Aaron, Aaron Chong, Chen Bainian, Grey, Hellyna NG, Marco A. Gutiérrez, Michael X. Grey, Yadu, Yadunund, Yadunund Vijay
