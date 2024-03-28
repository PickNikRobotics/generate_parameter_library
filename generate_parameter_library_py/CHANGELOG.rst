^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package generate_parameter_library_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.8 (2024-03-27)
------------------
* Restore functionality for mapped params with no struct name (`#185 <https://github.com/PickNikRobotics/generate_parameter_library/issues/185>_`)
* add # flake8: noqa to template (`#177 <https://github.com/PickNikRobotics/generate_parameter_library/issues/177>`_)
* Fix newline issue (`#176 <https://github.com/PickNikRobotics/generate_parameter_library/issues/176>`_)
  * fix new line rendering for Python
* Support nested mapped parameters (`#166 <https://github.com/PickNikRobotics/generate_parameter_library/issues/166>`_)
* Contributors: Paul Gesel, Sebastian Castro

0.3.7 (2024-01-12)
------------------
* fix Python runtime parameters bug (`#165 <https://github.com/PickNikRobotics/generate_parameter_library/issues/165>`_)
* markdown/rst: Support __map\_ and nested parameters (`#164 <https://github.com/PickNikRobotics/generate_parameter_library/issues/164>`_)
  * Support nested parameters and maps for RST
  * Add read_only info to RST template
  * Use map parameters for AutoDocumentation
  * Add test for markdown generation
  * Use jinja indentation instead of regex
  * Add dynamic_parameters also to DefaultConfig
  * Proper fromat of default yaml config
  * Proper format of the default yaml file
  * Fix imported function names
  * Add a jinja2 filter to create valid multiline c++ string literals
  * Handle empty strings for descriptions
* parse_yaml.py: keep the trailing newline (`#159 <https://github.com/PickNikRobotics/generate_parameter_library/issues/159>`_)
* Fix rolling CI (`#162 <https://github.com/PickNikRobotics/generate_parameter_library/issues/162>`_)
* parse_yaml.py: fix issue with typeguard (`#154 <https://github.com/PickNikRobotics/generate_parameter_library/issues/154>`_)
* Bugfixes on Python code generation (`#152 <https://github.com/PickNikRobotics/generate_parameter_library/issues/152>`_)
  * Fix raising of validation error
  * Fix inverse logic of 'unique' validator
* Fix sentence for size_lt (`#150 <https://github.com/PickNikRobotics/generate_parameter_library/issues/150>`_)
* Fix typeguard error (`#149 <https://github.com/PickNikRobotics/generate_parameter_library/issues/149>`_)
  For typeguard==3.0.2, `default_value: any` raised a `TypeError: isinstance() arg 2 must be a type, a tuple of types, or a union` during colcon build of a Python node using this library.
  Using the `Any` from the `typing` module fixed this issue.
  Yes, this problem does not occur with the typeguard version provided by apt (2.2.2), but could also occur if Ubuntu bumps that version or for other users with a newer local pip install of typeguard.
* Remove `my_parameter` from python template (`#146 <https://github.com/PickNikRobotics/generate_parameter_library/issues/146>`_)
* Contributors: Bruno-Pier, Christoph Fröhlich, Jan Gutsche, Matthias Schoepfer, Paul Gesel, agonzat

* fix Python runtime parameters bug (`#165 <https://github.com/PickNikRobotics/generate_parameter_library/issues/165>`_)
* markdown/rst: Support __map\_ and nested parameters (`#164 <https://github.com/PickNikRobotics/generate_parameter_library/issues/164>`_)
  * Support nested parameters and maps for RST
  * Add read_only info to RST template
  * Use map parameters for AutoDocumentation
  * Add test for markdown generation
  * Use jinja indentation instead of regex
  * Add dynamic_parameters also to DefaultConfig
  * Proper fromat of default yaml config
  * Proper format of the default yaml file
  * Fix imported function names
  * Add a jinja2 filter to create valid multiline c++ string literals
  * Handle empty strings for descriptions
* parse_yaml.py: keep the trailing newline (`#159 <https://github.com/PickNikRobotics/generate_parameter_library/issues/159>`_)
* Fix rolling CI (`#162 <https://github.com/PickNikRobotics/generate_parameter_library/issues/162>`_)
* parse_yaml.py: fix issue with typeguard (`#154 <https://github.com/PickNikRobotics/generate_parameter_library/issues/154>`_)
* Bugfixes on Python code generation (`#152 <https://github.com/PickNikRobotics/generate_parameter_library/issues/152>`_)
  * Fix raising of validation error
  * Fix inverse logic of 'unique' validator
* Fix sentence for size_lt (`#150 <https://github.com/PickNikRobotics/generate_parameter_library/issues/150>`_)
* Fix typeguard error (`#149 <https://github.com/PickNikRobotics/generate_parameter_library/issues/149>`_)
  For typeguard==3.0.2, `default_value: any` raised a `TypeError: isinstance() arg 2 must be a type, a tuple of types, or a union` during colcon build of a Python node using this library.
  Using the `Any` from the `typing` module fixed this issue.
  Yes, this problem does not occur with the typeguard version provided by apt (2.2.2), but could also occur if Ubuntu bumps that version or for other users with a newer local pip install of typeguard.
* Remove `my_parameter` from python template (`#146 <https://github.com/PickNikRobotics/generate_parameter_library/issues/146>`_)
* Contributors: Bruno-Pier, Christoph Fröhlich, Jan Gutsche, Matthias Schoepfer, Paul Gesel, agonzat

0.3.6 (2023-07-31)
------------------

0.3.5 (2023-07-28)
------------------
* Remove documentation of deprecated validator functions (`#135 <https://github.com/PickNikRobotics/generate_parameter_library/issues/135>`_)
* Contributors: Tyler Weaver

0.3.4 (2023-07-24)
------------------
* Use node logger (`#132 <https://github.com/PickNikRobotics/generate_parameter_library/issues/132>`_)
* Fixed incorrect usage of fmt, causing compile errors (`#127 <https://github.com/PickNikRobotics/generate_parameter_library/issues/127>`_)
* Add pyyaml as dependency (`#123 <https://github.com/PickNikRobotics/generate_parameter_library/issues/123>`_)
* Fix Python install (`#122 <https://github.com/PickNikRobotics/generate_parameter_library/issues/122>`_)
* Use correct syntax highlighting (`#121 <https://github.com/PickNikRobotics/generate_parameter_library/issues/121>`_)
* ReST template (`#119 <https://github.com/PickNikRobotics/generate_parameter_library/issues/119>`_)
* Fix empty arguments case and add custom validator prompt (`#117 <https://github.com/PickNikRobotics/generate_parameter_library/issues/117>`_)
* Update markdown template (`#116 <https://github.com/PickNikRobotics/generate_parameter_library/issues/116>`_)
* Fix output in current folder (`#115 <https://github.com/PickNikRobotics/generate_parameter_library/issues/115>`_)
* Generate Markdown Docs for Parameters  (`#111 <https://github.com/PickNikRobotics/generate_parameter_library/issues/111>`_)
* Add Python support for generate_parameter_library (`#110 <https://github.com/PickNikRobotics/generate_parameter_library/issues/110>`_)
  Co-authored-by: Tyler Weaver <tyler@picknik.ai>
* Update .pre-commit-config.yaml (`#108 <https://github.com/PickNikRobotics/generate_parameter_library/issues/108>`_)
* Log when a parameter is set (`#106 <https://github.com/PickNikRobotics/generate_parameter_library/issues/106>`_)
* Contributors: Christoph Fröhlich, Paul Gesel, Tyler Weaver, mosfet80, sprenger120

0.3.3 (2023-04-13)
------------------
* Fix Parameter Descriptor Incorrectly Populating Range Constraints for size_lt and size_gt (`#105 <https://github.com/PickNikRobotics/generate_parameter_library/issues/105>`_)
* Contributors: Chance Cardona

0.3.2 (2023-04-12)
------------------
* Populate Range Constraints in Parameter Descriptors from Validation Functions (`#103 <https://github.com/PickNikRobotics/generate_parameter_library/issues/103>`_)
* Mark deprecated rsl method and propose alternative in the docs. (`#102 <https://github.com/PickNikRobotics/generate_parameter_library/issues/102>`_)
* Allow none type (`#99 <https://github.com/PickNikRobotics/generate_parameter_library/issues/99>`_)
* Fixed tests never failing although file not found (`#101 <https://github.com/PickNikRobotics/generate_parameter_library/issues/101>`_)
* Contributors: Chance Cardona, Dr. Denis, GuiHome

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
