^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hector_nav_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.1 (2021-01-15)
------------------

0.5.0 (2020-12-17)
------------------
* Moved hector_geotiff launch files to separate package to solve cyclic dependency.
  Clean up for noetic release.
* Bump CMake version to avoid CMP0048 warning
* Contributors: Marius Schnaubelt, Stefan Fabian

0.4.1 (2020-05-15)
------------------

0.3.6 (2019-10-31)
------------------

0.3.5 (2016-06-24)
------------------

0.3.4 (2015-11-07)
------------------
* hector_nav_msgs: removed yaw member from GetNormal response
  yaw is implicitly given by the normal vector
* Contributors: Dorothea Koert

0.3.3 (2014-06-15)
------------------
* added GetNormal service, that returns normal and orientation of an occupied cell
* Contributors: Dorothea Koert

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-10-09)
------------------
* added changelogs

0.3.0 (2013-08-08)
------------------
* catkinized hector_slam
