^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package generate_parameter_library
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2024-03-27)
------------------
* use python_install_dir (`#178 <https://github.com/PickNikRobotics/generate_parameter_library/issues/178>`_)
* Update CMakeLists.txt (`#173 <https://github.com/PickNikRobotics/generate_parameter_library/issues/173>`_)
* Contributors: Christoph Fröhlich, Paul Gesel

0.3.7 (2024-01-12)
------------------
* Enable generate_parameter_module through ament_cmake_python (`#161 <https://github.com/PickNikRobotics/generate_parameter_library/issues/161>`_)
* Contributors: Paul Gesel

0.3.6 (2023-07-31)
------------------

0.3.5 (2023-07-28)
------------------

0.3.4 (2023-07-24)
------------------
* Add Python support for generate_parameter_library (`#110 <https://github.com/PickNikRobotics/generate_parameter_library/issues/110>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Contributors: Paul Gesel

0.3.3 (2023-04-13)
------------------

0.3.2 (2023-04-12)
------------------

0.3.1 (2023-02-01)
------------------
* Add keyword INTERFACE to fix build error 'ar: no archive members specified' since the generated target is header-only (`#93 <https://github.com/PickNikRobotics/generate_parameter_library/issues/93>`_)
* Make it easy for users to override (`#92 <https://github.com/PickNikRobotics/generate_parameter_library/issues/92>`_)
* Contributors: Tyler Weaver, light-tech

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
* Depend on python package (`#75 <https://github.com/PickNikRobotics/generate_parameter_library/issues/75>`_)
* Drop requirement for CMake to 3.16 (`#73 <https://github.com/PickNikRobotics/generate_parameter_library/issues/73>`_)
* Added -- for ros-args to find (`#71 <https://github.com/PickNikRobotics/generate_parameter_library/issues/71>`_)
* Contributors: Michael Wrock, Tyler Weaver

0.2.5 (2022-09-20)
------------------
* 🚀 Add cmake macros for using tests with example yaml files. 🤖 (`#57 <https://github.com/PickNikRobotics/generate_parameter_library/issues/57>`_)
  Co-authored-by: Tyler Weaver <maybe@tylerjw.dev>
* Contributors: Denis Štogl

0.2.4 (2022-08-19)
------------------
* 0.2.3
* Contributors: Tyler Weaver

0.2.3 (2022-08-05)
------------------

0.2.2 (2022-08-03)
------------------

0.2.1 (2022-08-02)
------------------

0.2.0 (2022-08-01)
------------------
* Fixed length arrays (`#44 <https://github.com/PickNikRobotics/generate_parameter_library/issues/44>`_)
* Change package name (`#40 <https://github.com/PickNikRobotics/generate_parameter_library/issues/40>`_)
* Cleaup cmake paths (`#38 <https://github.com/PickNikRobotics/generate_parameter_library/issues/38>`_)
* parameter validators interface library (`#32 <https://github.com/PickNikRobotics/generate_parameter_library/issues/32>`_)
* Support exporting parameter libraries (`#28 <https://github.com/PickNikRobotics/generate_parameter_library/issues/28>`_)
* Contributors: Paul Gesel, Tyler Weaver

0.1.0 (2022-07-27)
------------------
* CMake to generate ROS parameter library.
* Contributors: Paul Gesel, Tyler Weaver
