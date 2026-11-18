^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* Object list: one ``color`` for all markers of a node replaces ``body.color``,
  ``velocity.color``, ``body_axes.{x,y,z}_color``, ``covariance.{position,orientation}_color``
  and ``label.color``; the colour identifies the source, the namespace the element. New
  ``body.alpha`` (default 0.5) sets the opacity of the boxes relative to ``color``.
* One object list node visualizes several object lists: ``sources`` names them, and each source
  ``<name>`` has ``<name>.input_topic``, ``<name>.output_topic`` and ``<name>.color``; the
  markers of every source stay on their own topic. The other parameters are shared. Without
  ``sources`` the node behaves as before (``object_list`` -> ``object_markers``). Repeated
  sources and shared output topics fail the start of the node.
* New ``config/ufil_sources.yaml`` with the sources of the UFIL examples: OSN blue, OBU green,
  CAM (``rsu_cam``) violet, RSU orange, SSL yellow; the default colour is grey.
* Object list: the label is now ``<class> #<id>  ∃<existence probability>%`` (e.g.
  ``🚶 #42  ∃93%``), the class as an icon; ``label.icons: false`` shows the class name instead.
  ``label.text_size`` defaults to 0.8, the size of the lanelet ids.
* Both nodes are composable components (``ufil_visualization::LaneletMapMarkerArrayNode``,
  ``ufil_visualization::ObjectListMarkerArrayNode``); the ``lanelet_map_marker_array_node`` and
  ``object_list_marker_array_node`` executables are generated from them. Default node names
  changed to ``lanelet_map_marker_array`` and ``object_list_marker_array``.
* The marker geometry (lanelet triangulation, crosswalk stripes, object boxes, arrows,
  covariance ellipses, labels) moved into a ROS-independent core library.
* Parameters are grouped, read-only, documented and validated; invalid values (e.g. colours
  without four entries in [0, 1], non-positive sizes) fail the start of the node. Renamed:

  * lanelet map: ``lanelet_*_color`` / ``lanelet_*_z`` -> ``lanelet.*``, ``lanelet_id_*`` and
    ``show_lanelet_ids`` -> ``lanelet_ids.*``, ``road_marking_*`` and ``stop_line_width`` ->
    ``road_marking.*``, ``crosswalk_*`` -> ``crosswalk.*``, ``area_*`` -> ``area.*``.
    Removed ``area_scale`` (it scaled the filled areas along x only) and ``use_transient_local``
    (the map is published once, so it is always transient local).
  * object list: ``object_timeout`` -> ``marker_lifetime`` (it was declared but unused; it now
    sets the marker lifetime), colours and sizes grouped into ``body.*``, ``velocity.*``,
    ``body_axes.*``, ``covariance.*`` and ``label.*``. Removed ``input_topic`` / ``output_topic``.

* Topics are fixed names that are remapped instead of parameters: the object list node
  subscribes ``object_list`` and publishes ``object_markers``; the lanelet map node publishes
  ``map_markers`` (``map`` collides with the usual nav_msgs/OccupancyGrid topic).
* New launch files ``launch/lanelet_map.launch.xml`` and ``launch/object_list.launch.xml`` with
  ``use_sim_time`` (default false), ``params_file``, ``namespace``, ``node_name`` and topic
  arguments, loading ``config/lanelet_map.yaml`` / ``config/object_list.yaml`` (formerly
  ``lanelet_map_marker_array.yaml`` / ``object_list_marker_array.yaml``, now ``/**`` scoped).
  ``lanelet_map_marker_array.launch.xml`` and ``object_list_marker_array.launch.xml`` are kept
  with their arguments as wrappers; the former still publishes on ``map``.
* An empty ``map_path`` no longer aborts the lanelet map node; it logs a warning and publishes
  nothing. Recoverable Lanelet2 parser errors are logged instead of aborting the node.
* Fixed: the object list node published a separate DELETEALL array before every marker array,
  which made the markers flicker; DELETEALL is now the first marker of the same array.
* Fixed: covariance ellipses were drawn for unknown (negative) variances.
* Area markers are ordered deterministically by subtype.
* Added unit tests for the core library, parameters, conversions and components.
* Removed the unused ``tf2``, ``tf2_ros`` and ``tf2_geometry_msgs`` dependencies.
* Third-party code: ``src/earcut.hpp`` is mapbox/earcut.hpp (Copyright (c) 2015, Mapbox, ISC
  license). Its license header was restored and it is no longer installed as a public header.
