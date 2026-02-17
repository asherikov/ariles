Ariles
======

<table>
  <tr>
    <th></th>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/head_2">HEAD v2</a>
    </td>
  </tr>
  <tr>
    <th>CI status</th>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/actions?query=workflow%3A.github%2Fworkflows%2Fhead_2.yml+branch%3Ahead_2">
        <img src="https://github.com/asherikov/ariles/actions/workflows/.github/workflows/head_2.yml/badge.svg?branch=head_2" alt="Build Status">
        </a>
    </td>
  </tr>
  <tr>
    <th>latest</th>
    <td align="center">
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fjammy;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fjammy;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
        <br />
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fnoble;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fnoble;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
  </tr>
  <tr>
    <th>legacy</th>
    <td align="center">
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fbionic;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fbionic;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
        <br />
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Ffocal;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Ffocal;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
        </a>
  </tr>
</table>



Contents
========
* [Links](#links)
* [Introduction](#intro)
* [Minimal example](#example)
* [Visitors](#visitors)
* [Supported types](#types)
* [Dependencies and compilation](#compilation)
* [Advanced features](#advanced)
* [Tips](#tips)
* [Related software](#related)
* [Legacy](#legacy)


<a name="links"></a>
Links
=====
* Documentation (Doxygen): <https://asherikov.github.io/ariles/2/>
* GitHub: <https://github.com/asherikov/ariles>


<a name="intro"></a>
Introduction
============

Loosely speaking, `ariles` is a C++ reflection library, i.e., it provides
meta-programming APIs for implementation of class visitors (processors). It
also provides a number of (de)serializers based on these APIs, e.g., `YAML`,
`JSON`, `XML`, `ROS` parameter server, `ROS2` parameters; and serialization
wrappers for some types, e.g., `STL` containers, smart pointers, `Eigen`
matrices, etc.

Unlike many modern C++ reflection/serialization libraries (see a list of
related software below) `ariles` uses macro approach for injecting code. This
is not an archaism, but a design choice which enables a different set of
features: virtual serialization methods, no constraints on class construction
or inheritance, etc.


<a name="uses"></a>
Use cases
---------

1. Parsing and generation of configuration files. Unlike some common
   serialization libraries, e.g., `boost::serialization`, `ariles` tries to be
   flexible while parsing by:
    - silently ignoring unused entries (if possible),
    - ignoring ordering of entries (if possible),
    - not discriminating attributes from childs in `XML`,
    - optionally ignoring missing entries.

2. Conversion between different formats, for example, `YAML` <-> `ROS`
   parameter server. Note that the conversion is not data-agnostic, i.e., the
   complete data structure must be represented in C++ code.

3. Flattening of a class hierarchy to a list of name-value pairs
   (string-double), which is useful for collection of time-series data, see
   telemetry collection library <https://github.com/asherikov/intrometry>.

4. Exporting of numerical data to an `Octave` or `python `script for debugging
   purposes.

5. Implementation of parsers for specific data formats, e.g., a proof of
   concept `URDF` parser <https://github.com/asherikov/ariles_urdf>.


<a name="example"></a>
Minimal example
===============


Class [`./tests/api_v2/types/minimal.h`]:
```
class Configurable : public ariles2::DefaultBase
{
    #define ARILES2_DEFAULT_ID "ConfigurableEntryName" // optional, defaults to 'ariles'
    #define ARILES2_ENTRIES(v) \
        ARILES2_TYPED_ENTRY(v, integer_member, int)
    #include ARILES2_INITIALIZE
};
```

Serialization:
```
Configurable configurable;
configurable.integer_member = 10;
ariles2::apply<ariles2::yaml_cpp::Writer>("config.yaml", configurable);
ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, configurable);
```

Result:
```
ConfigurableEntryName:
    integer_member: 10
```

Deserialization:
```
ariles2::apply<ariles2::yaml_cpp::Reader>("config.yaml", configurable);
```

Conversion:
```
// read class from a file
ariles2::apply<ariles2::yaml_cpp::Reader>("config.yaml", configurable);
// dump class to ROS parameter server
ariles2::apply<ariles2::rosparam::Writer>(nh, configurable, "/some_namespace/");
```

See demo for more examples: <https://asherikov.github.io/ariles/2/DEMO.html>
[`./ariles/tests/api_v2/demo_api_v2.cpp`]



<a name="visitors"></a>
Visitors
========

`ariles` includes a number of optional visitors that support various data
representation formats, in particular:

* `YAML` via `yaml-cpp`:
  <https://asherikov.github.io/ariles/2/group__yaml__cpp.html>.

* `msgpack` via `msgpack-c`:
  <https://asherikov.github.io/ariles/2/group__msgpack.html>.

* `JSON` via `nlohmann_json`, with optional Jsonnet preprocessing:
  <https://asherikov.github.io/ariles/2/group__nlohmann__json.html> and
  <https://asherikov.github.io/ariles/2/group__jsonnet.html>.

* `XML` via `PugiXML`:
  <https://asherikov.github.io/ariles/2/group__pugixml.html>

* `Octave` script, output only, no dependencies:
  <https://asherikov.github.io/ariles/2/group__octave.html>

* `Python` script, output only, no dependencies, uses numpy for vectors and matrices:
  <https://asherikov.github.io/ariles/2/group__python.html>

* `ROS` parameter server, via `ROS` libs:
  <https://asherikov.github.io/ariles/2/group__rosparam.html>

* A set of flattened key-value pairs, output only, no dependencies:
  <https://asherikov.github.io/ariles/2/group__namevalue2.html>

* `graphviz` dot files for diagram generation:
  <https://asherikov.github.io/ariles/2/group__graphviz.html>

* `ROS2` parameters, via `rclcpp` lib:
  <https://asherikov.github.io/ariles/2/group__ros2param.html> `ROS2`
  parameters are not designed to fully reflect yaml structure as explained here
  <https://github.com/ros2/rcl/issues/463>, so while `ariles` can dump and read
  anything, there are certain workarounds in place that are described in more
  details in the `ROS2` demo [`./tests/api_v2/demo_api_v2_ros2.cpp`]
  <https://asherikov.github.io/ariles/2/DEMO_ROS2.html>.

There are also a few utility visitors, e.g.,

* `compare` for class comparison;
* `copyto` for copying data to non-`ariles` classes;
* `copyfrom` for copying data from non-`ariles` classes.


The complete list of modules is available at
<https://asherikov.github.io/ariles/2/modules.html>



<a name="types"></a>
Supported data types
====================

`ariles` provides serialization wrappers for the following types:

* Fundamental types: integers, floats, booleans.
* STL classes: `std::string`, `std::vector`, `std::map`, `std::pair`,
  `std::shared_ptr`, `std::unique_ptr`, `std::array`, `std::deque`,
  `std::list`, `std::set`, `std::unordered_map`, `std::unordered_set`,
  `std::chrono::duration`, `std::chrono::time_point`, `std::optional`,
  `std::filesystem::path`, `std::tuple`.
* `Eigen` types: matrices, transforms, quaternions.
* `Boost` classes: `boost::optional`, `boost::movelib::unique_ptr`. `boost::shared_ptr`.
* Better enums -> <https://github.com/aantron/better-enums>.



<a name="compilation"></a>
Dependencies and compilation
============================

The library is organized in a core package and a set of dependent visitor
packages. It can be built in two different ways:

- using plain cmake with features configured by cmake options -- the project is
  located in `ariles` subdirectory;
- using catkin/colcon in a ROS1/ROS2 workspace, in which case features are
  selected by building corresponding wrapper packages, see `ariles2_*_ws`
  directories.


Core dependencies
-----------------

- `cmake` >= 3.13
- `C++17` compatible compiler
- `boost`



<a name="advanced"></a>
Advanced features
=================

Sloppy maps and pairs
---------------------

`std::pair` is by default represented in the following way:
```
    my_pair:
        first: first_value
        second: second_value
```
The default behavior is generic and robust, but some users prefer to use a more
compact form provided that the first value is represented by `std::string`:
```
    my_pair:
        first_value: second_value
```
The alternative behavior can be enabled using `sloppy_pairs_` flag in
`serialization::Parameters`. Note that in general it is up to the user to
ensure that `first_value` is a valid map key in the target serialization
format. `std::map` with string keys are handled in a similar way when
`sloppy_maps_` parameter is enabled. If you are inheriting from a "sloppy"
`ariles` base classes, e.g., `ariles2::SloppyBase` (see `ariles2/extra.h`
header), both of these flags are enabled by default.


"Any": polymorphic configurations
---------------------------------

`ariles2::Any2` class defined in `ariles2/types.h` provides functionality similar
to `protobuf::Any`: it allows automatic instantiation and configuration reading
of user-defined classes based on their string ids. See
`tests/api_v2/types/any.h` for an example.


<a name="tips"></a>
Tips
====

- If you are having problems figuring out the correct configuration file layout
  for an `ariles` class, try writing it first to get an example. `ariles` can
  parse its own output.

- It is not possible to make some members optional and require the rest.
  However, you can wrap these members with smart pointers or `boost::optional`,
  which may be uninitialized. `ariles` automatically adds `is_null` flag for
  each of such variables in configuration files.

- Configuration file readers are defined as aggregate visitors that
  sequentially do the following: (1) reset values to defaults, (2) read values,
  (3) finalize values. In some cases you may want to skip (1) and (3), which
  can be achieved by using `ns_ros2param::Reader` (pick desired reader
  namespace) directly or defining your own aggregate visitor. It would be
  necessary when reading data from multiple partially initialized sources to
  avoid resets.


<a name="related"></a>
Related software
================

* <https://github.com/PickNikRobotics/rosparam_shortcuts>: a set of wrapper
  functions to read individual parameters from ROS parameter server. This tool
  serves pretty much the same purpose as `ariles2::rosparam::Reader`, but its
  functionality is more limited.

* <https://billyquith.github.io/ponder/>: C++14 reflection library, supports
  serialization to XML and JSON. Unlike `ariles` it is more focused on
  reflection per se rather than applications, for example, it allows to set
  value by string name of a class member, handles class methods, etc. `Ponder`
  does not rely as much on preprocessor macro, but is more verbose.

* <https://github.com/bytemaster/boost_reflect>: discontinued C++ reflection
  library, similar to `ponder`. Partially inspired `ariles` 2.x.x API.

* <https://github.com/apolukhin/magic_get> (aka `pfr`): C++14 library providing
  tuple like methods for aggregate initializable structures. Addresses a
  somewhat different but related problem.

* <https://github.com/getml/reflect-cpp>: a powerful C++20 POD-type reflection
  library.

* Serialization libraries, e.g., `boost::serialization`,
  <https://github.com/USCiLab/cereal>.

* A library with similar functionality in C++17
  <https://github.com/injae/serdepp>.


<a name="legacy"></a>
Legacy
======

Version 1
---------

- Source code: <https://github.com/asherikov/ariles/tree/head_1>
- ROS workspace branch: <https://github.com/asherikov/ariles/tree/pkg_ros>
- Documentation: <https://asherikov.github.io/ariles/1/>
- Migration guide: <https://asherikov.github.io/ariles/2/md_doc_migration_1to2.html>

Version 2
---------

Refer to changelog for more details.

- APIv1 has been removed.
- Dropped support for legacy Ubuntu versions, old C++ standards, old cmake
  versions in minor releases.
- Dropped ROS workspace branches:
    - <https://github.com/asherikov/ariles/tree/pkg_catkin_2> up to version 2.2.0.
    - <https://github.com/asherikov/ariles/tree/pkg_ws_2> up to version 2.7.0.

Version 3 (planned)
-------------------

To be removed:

- `ariles2::namevalue`: replaced by `ariles2::namevalue2`
- `ariles2::Any`: replaced by `ariles2::Any2`
- `rosparam`: ROS1 is EOL.
- `rapidjson`: poorly maintained, Ubuntu packages are based on an old buggy
  release <https://github.com/Tencent/rapidjson/issues/718>. Replaced by
  `nlohmann_json`.
