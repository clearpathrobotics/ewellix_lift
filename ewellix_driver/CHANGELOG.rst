^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ewellix_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.1 (2025-10-22)
------------------
* Thread sleep
* Contributors: Luis Camero

0.2.0 (2025-08-20)
------------------
* Update license in package.xml
* Merge pull request `#7 <https://github.com/clearpathrobotics/ewellix_lift/issues/7>`_ from Ayush1285/feat/last-position
  Update the initial state of the lift.
* Merge pull request `#4 <https://github.com/clearpathrobotics/ewellix_lift/issues/4>`_ from Ayush1285/humble
  Encoder limits as configurable parameters
* Fetch prev position
* Added encoder limit struct
* Added upper and lower limits as member variables
* Added encoder lower and upper limits params.
* Contributors: Ayush Singh, Luis Camero

0.1.0 (2025-01-15)
------------------
* Removed logging
* Remove INFO logs
* Switch to stop before execute motion
* Add recovery
* Add Out and In execute functions
* Catch overcurrent
* Remove INFO logs
* Switch to stop before execute motion
* Add recovery
* Add Out and In execute functions
* Catch overcurrent
* Duplicate joint count to account for mimic joints
* Use ewellix_interfaces instead of std
* Add new interfaces
* Update default port
* Reset package versions to 0.0.0
* Add custom messages
* Fix parameters
* Initialize vectors in each line
* Use this to reference member methods
* Remove unused source file
* Refactor Ewellix node to use asynchronous thread
* Remove commented out code
* Use CycleData2 and asyncThread
* Use classes instead of structs
* Update hardware_interface to use DataCycle2
* Use DataCycle2 in node
* Add DataCycle2
* Initial add of Ewellix driver
* Contributors: Luis Camero
