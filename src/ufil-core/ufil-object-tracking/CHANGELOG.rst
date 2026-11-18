^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_object_tracking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* C++20 is a usage requirement of the ``ufil_object_tracking`` target instead of a global
  setting; warnings are only enabled for the tests.
* Fixed stale tests (``Scalar`` is ``double``, default existence probability 1.0, JSON keys) and
  initialized ``HistoryEntry::occluded_`` in the two-argument constructor.
