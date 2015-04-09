^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package adapy
^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2015-04-08)
------------------
* fixed bug in adarobot that could not load the tsr library
  renamed robot name to ADA in tsr.yaml files
* Changed default flag to True
* Adapy now supports loading urdf model
* Fixed joint-limits
* Removed named planner for now
* Updated adarobot to load planners in the same way as in herbpy master
* changed planner sequence to be similar to herbpy
* Moved test.py to scripts/.
* Deleted .pyc files and added them go .gitignore.
* Added console.py to AdaPy, to mimic HerbPy.
* added named configurations
* Contributors: Michael Koval, Stefanos Nikolaidis
