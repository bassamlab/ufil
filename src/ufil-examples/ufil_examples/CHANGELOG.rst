^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ufil_examples
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2026-10-30)
------------------
* The example bringup packages are exec dependencies instead of build dependencies; the
  package no longer calls ``find_package`` for them.
* ``ufil_examples_uofa`` stays excluded (documented in ``package.xml``): the UofA example is
  removed from the public GitHub mirror of the workspace.
* Package metadata: description, version 0.2.0, lint tests.
