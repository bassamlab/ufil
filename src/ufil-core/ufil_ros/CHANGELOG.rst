^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* ``visibility_control.h`` was renamed to ``visibility_control.hpp`` and is now used: the
  library is built with hidden visibility and exports the functions marked ``UFIL_ROS_PUBLIC``.
  The explicit specializations of the conversion templates are declared in ``ufil_ros.hpp``.
* ``to_json``: numbers keep their full precision (they were rounded to 6 decimals), NaN and
  infinity become ``null`` and ``frame_id`` is escaped, so the output is always valid JSON.
* C++20 and warnings are set on the target instead of globally; the copyright and cpplint
  checks are enabled.
