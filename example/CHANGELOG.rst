^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package generate_parameter_library_example
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2025-01-13)
------------------
* Change header install path (`#213 <https://github.com/PickNikRobotics/generate_parameter_library/issues/213>`_)
* Contributors: Auguste Bourgois

0.3.9 (2024-10-27)
------------------
* Add "additional_constraints" support (`#221 <https://github.com/PickNikRobotics/generate_parameter_library/issues/221>`_)
* Use int64_t instead of int for parameter integer range, fixes `#199 <https://github.com/PickNikRobotics/generate_parameter_library/issues/199>`_ (`#214 <https://github.com/PickNikRobotics/generate_parameter_library/issues/214>`_)
* Add Non-Blocking try_get_params Function for Real-Time Control Systems (`#205 <https://github.com/PickNikRobotics/generate_parameter_library/issues/205>`_)
* Drop yaml brackets for consistency and readability (`#203 <https://github.com/PickNikRobotics/generate_parameter_library/issues/203>`_)
* Contributors: Auguste Bourgois, David Revay, KentaKato, Tim Clephas

0.3.8 (2024-03-27)
------------------
* Restore functionality for mapped params with no struct name (`#185 <https://github.com/PickNikRobotics/generate_parameter_library/issues/185>_`)
* Fix newline issue (`#176 <https://github.com/PickNikRobotics/generate_parameter_library/issues/176>`_)
  * fix new line rendering for Python
* Support nested mapped parameters (`#166 <https://github.com/PickNikRobotics/generate_parameter_library/issues/166>`_)
* Contributors: Paul Gesel, Sebastian Castro

0.3.7 (2024-01-12)
------------------
* Split example/README.md into C++ and Python version; updated content (`#138 <https://github.com/PickNikRobotics/generate_parameter_library/issues/138>`_)
* Contributors: chriseichmann

0.3.6 (2023-07-31)
------------------

0.3.5 (2023-07-28)
------------------
* Remove documentation of deprecated validator functions (`#135 <https://github.com/PickNikRobotics/generate_parameter_library/issues/135>`_)
* Contributors: Tyler Weaver

0.3.4 (2023-07-24)
------------------
* Add Python support for generate_parameter_library (`#110 <https://github.com/PickNikRobotics/generate_parameter_library/issues/110>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Contributors: Paul Gesel

0.3.3 (2023-04-13)
------------------

0.3.2 (2023-04-12)
------------------
* Populate Range Constraints in Parameter Descriptors from Validation Functions (`#103 <https://github.com/PickNikRobotics/generate_parameter_library/issues/103>`_)
* Mark deprecated rsl method and propose alternative in the docs. (`#102 <https://github.com/PickNikRobotics/generate_parameter_library/issues/102>`_)
* Contributors: Chance Cardona, Dr. Denis

0.3.1 (2023-02-01)
------------------
* Make it easy for users to override (`#92 <https://github.com/PickNikRobotics/generate_parameter_library/issues/92>`_)
* Contributors: Tyler Weaver

0.3.0 (2022-11-15)
------------------
* Migrate from parameter_traits to RSL (take 2) (`#91 <https://github.com/PickNikRobotics/generate_parameter_library/issues/91>`_)
* Contributors: Tyler Weaver

0.2.8 (2022-11-03)
------------------

0.2.7 (2022-10-28)
------------------
* Standardize cmake (`#79 <https://github.com/PickNikRobotics/generate_parameter_library/issues/79>`_)
* Contributors: Tyler Weaver

0.2.6 (2022-09-28)
------------------

0.2.5 (2022-09-20)
------------------
* 🈵 Support use of '_' in mapped parameters. (`#68 <https://github.com/PickNikRobotics/generate_parameter_library/issues/68>`_)
* Component node example (`#60 <https://github.com/PickNikRobotics/generate_parameter_library/issues/60>`_)
* Update README for example (`#63 <https://github.com/PickNikRobotics/generate_parameter_library/issues/63>`_)
* 🚀 Add cmake macros for using tests with example yaml files. 🤖 (`#57 <https://github.com/PickNikRobotics/generate_parameter_library/issues/57>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
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

0.2.1 (2022-08-02)
------------------
* Fix scientific notation (`#46 <https://github.com/PickNikRobotics/generate_parameter_library/issues/46>`_)
* Contributors: Paul Gesel

0.2.0 (2022-08-01)
------------------
* Create stack allocated struct (`#45 <https://github.com/PickNikRobotics/generate_parameter_library/issues/45>`_)
* Fixed length arrays (`#44 <https://github.com/PickNikRobotics/generate_parameter_library/issues/44>`_)
* Fixed size string no default bug (`#43 <https://github.com/PickNikRobotics/generate_parameter_library/issues/43>`_)
* static OK to fix ODR errors (`#41 <https://github.com/PickNikRobotics/generate_parameter_library/issues/41>`_)
* Change package name (`#40 <https://github.com/PickNikRobotics/generate_parameter_library/issues/40>`_)
* parameter validators interface library (`#32 <https://github.com/PickNikRobotics/generate_parameter_library/issues/32>`_)
* Validate fixed length Strings (`#33 <https://github.com/PickNikRobotics/generate_parameter_library/issues/33>`_)
* Fixed size strings (`#29 <https://github.com/PickNikRobotics/generate_parameter_library/issues/29>`_)
* Contributors: Paul Gesel, Tyler Weaver

0.1.0 (2022-07-27)
------------------
* Example usage of generate_parameter_library.
* Contributors: Paul Gesel, Tyler Weaver
