^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package generate_parameter_library_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2023-02-01)
------------------

0.3.0 (2022-11-15)
------------------
* Migrate from parameter_traits to RSL (take 2) (`#91 <https://github.com/PickNikRobotics/generate_parameter_library/issues/91>`_)
* Add missing dependency on PyYAML (`#89 <https://github.com/PickNikRobotics/generate_parameter_library/issues/89>`_)
* Contributors: Scott K Logan, Tyler Weaver

0.2.8 (2022-11-03)
------------------
* Use typing syntax which is compatible with Python 3.6 (`#87 <https://github.com/PickNikRobotics/generate_parameter_library/issues/87>`_)
* Use YAML loader which is compatible with PyYAML 3.12 (`#88 <https://github.com/PickNikRobotics/generate_parameter_library/issues/88>`_)
* Contributors: Scott K Logan

0.2.7 (2022-10-28)
------------------

0.2.6 (2022-09-28)
------------------
* Depend on python dependencies in package.xml (`#74 <https://github.com/PickNikRobotics/generate_parameter_library/issues/74>`_)
* Contributors: Tyler Weaver

0.2.5 (2022-09-20)
------------------
* 🈵 Support use of '_' in mapped parameters. (`#68 <https://github.com/PickNikRobotics/generate_parameter_library/issues/68>`_)
* Test validators and fix bugs (`#66 <https://github.com/PickNikRobotics/generate_parameter_library/issues/66>`_)
* Fix deadlock in update_dynamic_parameters (`#64 <https://github.com/PickNikRobotics/generate_parameter_library/issues/64>`_)
* Parameter prefix (`#55 <https://github.com/PickNikRobotics/generate_parameter_library/issues/55>`_)
* Fix example parameters (`#54 <https://github.com/PickNikRobotics/generate_parameter_library/issues/54>`_)
* Contributors: Denis Štogl, Paul Gesel, Tyler Weaver

0.2.4 (2022-08-19)
------------------
* INTEGER type (`#53 <https://github.com/PickNikRobotics/generate_parameter_library/issues/53>`_)
* 0.2.3
* Contributors: Tyler Weaver

0.2.3 (2022-08-05)
------------------

0.2.2 (2022-08-03)
------------------
* Add better error messages (`#48 <https://github.com/PickNikRobotics/generate_parameter_library/issues/48>`_)
* Lock mutex around modifying internal state of ParamListener (`#47 <https://github.com/PickNikRobotics/generate_parameter_library/issues/47>`_)
* Contributors: Paul Gesel, Tyler Weaver

0.2.1 (2022-08-02)
------------------
* Fix scientific notation (`#46 <https://github.com/PickNikRobotics/generate_parameter_library/issues/46>`_)
* Contributors: Paul Gesel

0.2.0 (2022-08-01)
------------------
* Create stack allocated struct (`#45 <https://github.com/PickNikRobotics/generate_parameter_library/issues/45>`_)
* Fixed length arrays (`#44 <https://github.com/PickNikRobotics/generate_parameter_library/issues/44>`_)
* Fixed size string no default bug (`#43 <https://github.com/PickNikRobotics/generate_parameter_library/issues/43>`_)
* Move fixed size string to parameter traits (`#42 <https://github.com/PickNikRobotics/generate_parameter_library/issues/42>`_)
* static OK to fix ODR errors (`#41 <https://github.com/PickNikRobotics/generate_parameter_library/issues/41>`_)
* Change package name (`#40 <https://github.com/PickNikRobotics/generate_parameter_library/issues/40>`_)
* parameter validators interface library (`#32 <https://github.com/PickNikRobotics/generate_parameter_library/issues/32>`_)
* Validate fixed length Strings (`#33 <https://github.com/PickNikRobotics/generate_parameter_library/issues/33>`_)
* Fixed size strings (`#29 <https://github.com/PickNikRobotics/generate_parameter_library/issues/29>`_)
* Use single namespace validators (`#26 <https://github.com/PickNikRobotics/generate_parameter_library/issues/26>`_)
* Validate strings and arrays for size (`#24 <https://github.com/PickNikRobotics/generate_parameter_library/issues/24>`_)
* Contributors: Paul Gesel, Tyler Weaver

0.1.0 (2022-07-27)
------------------
* Python to generate C++ ROS parameter library.
* Contributors: Paul Gesel, Tyler Weaver
