^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_mqtt
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The node is a composable component (``ufil_mqtt::MqttMessageConverterNode``); the
  ``mqtt_message_converter_node`` executable is generated from it. The default node name changed
  from ``mqtt_publisher_node`` to ``mqtt_message_converter``.
* The MQTT client moved into the ROS-independent ``ufil_mqtt::MqttClient`` library, which reports
  its connection state, counters and events to the component.
* Parameters renamed and grouped: ``protocol``, ``host``, ``port``, ``id`` became
  ``broker.protocol``, ``broker.host``, ``broker.port``, ``broker.client_id``; added
  ``broker.reconnect_interval``. ``mqtt_topic`` and ``message_type`` are unchanged. An empty
  ``broker.client_id`` (the default) uses the fully qualified node name.
* The ``source_topic`` parameter was replaced by the ``input`` topic, which is remapped by the
  launch files. The subscription uses the sensor data QoS (best effort), so it also receives from
  best effort publishers.
* Fixed: ``launch/mqtt.launch.xml`` set parameters the node did not declare (``mqtt_host``,
  ``mqtt_port``, ...), so all arguments except ``mqtt_topic`` and ``mqtt_use_sim_time`` were
  ignored. It is now a wrapper around the new ``launch/default.launch.xml`` with the same arguments;
  the unused ``mqtt_enable_stats`` argument was removed and the default ``mqtt_topic`` is
  ``ufil/object_list`` instead of ``test``.
* Fixed: a broker that was not reachable at start-up was never connected; connection attempts are
  now repeated every ``broker.reconnect_interval`` seconds, also after a lost connection.
* Fixed: TLS options were set for plain ``tcp`` connections; the connection state was shared
  between threads without synchronisation; invalid parameters called ``rclcpp::shutdown()``.
  Invalid parameters are now logged as fatal before the construction of the component fails.
* Forwarded messages and MQTT callbacks are no longer logged at INFO. Added diagnostics (connection
  state, sent / dropped / failed messages, last error), ``config/default.yaml`` and unit tests.
