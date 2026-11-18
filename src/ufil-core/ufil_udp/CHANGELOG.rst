^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_udp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The nodes are composable components (``ufil_udp::UdpReceiverNode``,
  ``ufil_udp::UdpSenderNode``); the ``udp_receive_node`` and ``udp_sender_node`` executables are
  generated from them. Node names (``udp_receiver``, ``udp_sender``) and topics (``udp_output``,
  ``udp_input``) are unchanged.
* Socket handling moved into a ROS-independent core library (``UdpReceiver``, ``UdpSender``).
* Sender parameter ``target_ip`` was renamed to ``target_address`` and also accepts IPv6
  addresses; invalid addresses and ports are rejected at start-up. All parameters are read-only.
  The launch arguments of ``udp_sender.launch.xml`` and ``udp_receiver.launch.xml`` are unchanged.
* Removed the ``enable_stats`` parameter and its periodic log. Packet and byte counters, errors and
  the age of the last packet are published on ``/diagnostics`` instead. The ``*_enable_stats``
  launch arguments are still accepted but ignored.
* Received packets are stamped with the reception time and carry the sender's address and port
  (``header.stamp``, ``address``, ``src_port`` were empty before). Empty datagrams are published.
* Fixed: datagrams larger than 4096 bytes were truncated.
* Fixed: the receive loop depended on ``rclcpp::ok()`` and the socket was cancelled from another
  thread while in use. The receive thread is now stopped and joined before the node is destroyed,
  so the receiver can be unloaded from a component container while packets arrive.
* Fixed: sending to an IPv4 broadcast address failed.
* Send and receive errors are logged throttled; a port that cannot be bound is logged as fatal
  before the construction of the component fails.
* Added ``launch/default.launch.xml`` (starts both nodes), ``config/default.yaml`` and unit tests.
