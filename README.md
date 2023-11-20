Ariles
======

<table>
  <tr>
    <th>branch</th>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/head_2">HEAD v2</a>
    </td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/pkg_catkin_2">pkg_catkin_2</a><br/>
        (ROS/catkin package)
    </td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/pkg_freebsd_2">pkg_freebsd_2</a><br/>
        (FreeBSD package)
    </td>
  </tr>
  <tr>
    <th>CI status</th>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/actions?query=workflow%3A.github%2Fworkflows%2Fhead_2.yml+branch%3Ahead_2">
        <img src="https://github.com/asherikov/ariles/workflows/.github/workflows/head_2.yml/badge.svg?branch=head_2" alt="Build Status">
        </a>
    </td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/actions?query=workflow%3A.github%2Fworkflows%2Fcatkin_2.yml+branch%3Apkg_catkin_2">
        <img src="https://github.com/asherikov/ariles/workflows/.github/workflows/catkin_2.yml/badge.svg?branch=pkg_catkin_2" alt="Build Status">
        </a>
    </td>
    <td align="center"></td>
  </tr>
  <tr>
    <th>package</th>
    <td align="center">
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fbionic;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fbionic;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
        <br />
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Ffocal;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Ffocal;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
        <a href="https://cloudsmith.io/~asherikov-aV7/repos/all/packages/detail/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fjammy;t=binary/">
        <img src="https://api-prd.cloudsmith.io/v1/badges/version/asherikov-aV7/all/deb/ariles2-core/latest/a=amd64;d=ubuntu%252Fjammy;t=binary/?render=true&show_latest=true" alt="Latest version of 'ariles' @ Cloudsmith">
        </a>
    </td>
    <td align="center"></td>
    <td align="center"></td>
  </tr>
</table>



Contents
========
* [Links](#links)
* [Introduction](#intro)
    * [Use cases](#uses)
* [Minimal example](#example)
* [Visitors](#visitors)
* [Supported types](#types)
* [Dependencies and compilation](#compilation)
* [Related software](#related)


<a name="links"></a>
Links
=====
* Documentation (Doxygen): https://asherikov.github.io/ariles/2/
* GitHub: https://github.com/asherikov/ariles
* Legacy 1.x.x version: https://github.com/asherikov/ariles/tree/head_1
  (migration guide https://asherikov.github.io/ariles/2/md_doc_migration_1to2.html)


<a name="intro"></a>
Introduction
============

Loosely speaking, `ariles` is a C++ reflection library, i.e., it provides
meta-programming APIs for implementation of class visitors (processors). It
also provides a number of (de)serializers based on these APIs, e.g., `YAML`,
`JSON`, `XML`, `ROS` parameter server; and serialization wrappers for some
types, e.g., `STL` containers, smart pointers, `Eigen` matrices, etc.


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
   (string-double), which is useful for collection of time-series data.

4. Exporting of numerical data to an `Octave` script for debugging purposes.

5. Implemetation of parsers for specific data formats, e.g., `URDF`.



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

See demo for more exaples: https://asherikov.github.io/ariles/2/DEMO.html
[`./tests/api_v2/demo_api_v2.cpp`]



<a name="visitors"></a>
Visitors
========

`ariles` includes a number of optional visitors that support various data
representation formats, in particular:

* `YAML` via `yaml-cpp`:
  https://asherikov.github.io/ariles/2/group__yaml__cpp.html.

* `msgpack` via `msgpack-c`:
  https://asherikov.github.io/ariles/2/group__msgpack.html.

* `JSON` via `RapidJSON`, with optional Jsonnet preprocessing:
  https://asherikov.github.io/ariles/2/group__rapidjson.html and
  https://asherikov.github.io/ariles/2/group__jsonnet.html.

* `XML` via `PugiXML`:
  https://asherikov.github.io/ariles/2/group__pugixml.html

* `Octave` script, output only, no dependencies:
  https://asherikov.github.io/ariles/2/group__octave.html

* `ROS` parameter server, via standard `ROS` libs:
  https://asherikov.github.io/ariles/2/group__ros.html

* A set of flattened key-value pairs, output only, no dependencies:
  https://asherikov.github.io/ariles/2/group__namevalue.html

* `graphviz` dot files for diagram generation:
  https://asherikov.github.io/ariles/2/group__graphviz.html


There are also a few utility visitors, e.g.,

* `compare` for class comparison;
* `copyto` for copying data to non-`ariles` classes;
* `copyfrom` for copying data from non-`ariles` classes.


The complete list of modules is available at
https://asherikov.github.io/ariles/2/modules.html



<a name="types"></a>
Supported data types
====================

`ariles` provides serialization wrappers for the follwing types:

* Fundametal types: integers, floats, booleans.
* Some STL classes (WIP): `std::string`, `std::vector`, `std::map`, `std::pair`, `std::shared_ptr`, `std::unique_ptr`.
* `Eigen` types: matrices, transforms, quaternions.
* `Boost` classes: `boost::optional`, `boost::movelib::unique_ptr`. `boost::shared_ptr`.
* Better enums -> https://github.com/aantron/better-enums.



<a name="compilation"></a>
Dependencies and compilation
============================

Dependencies
------------

- `cmake` >= 3.0
- `C++11` compatible compiler
- `boost`

Visitors and corresponding dependencies can be enabled or disabled via cmake
options, the same applies to data types which depend on external libraries.


Compilation with catkin
-----------------------

An example catkin package is provided in `pkg_catkin_2` branch of the main
repository -> https://github.com/asherikov/ariles/tree/pkg_catkin_2.



<a name="related"></a>
Related software
================

* https://github.com/PickNikRobotics/rosparam_shortcuts: a set of wrapper
  functions to read individual parameters from ROS parameter server. This tool
  serves pretty much the same purpose as `ariles2::rosparam::Reader`, but its
  functionality is more limited.

* https://billyquith.github.io/ponder/: C++14 reflection library, supports
  serialization to XML and JSON. Unlike `ariles` it is more focused on
  reflection per se rather than applications, for example, it allows to set
  value by string name of a class member, handles class methods, etc. `Ponder`
  does not rely as much on preprocessor macro, but is more verbose.

* https://github.com/bytemaster/boost_reflect: discontinued C++ reflection
  library, similar to `ponder`. Partially inspired `ariles` 2.x.x API.

* https://github.com/apolukhin/magic_get (aka `pfr`): C++14 library providing
  tuple like methods for aggregate initializable structures. Addresses a
  somewhat different but related problem.

* Serialization libraries, e.g., `boost::serialization`,
  https://github.com/USCiLab/cereal.

* A library with similar functionality in C++17
  https://github.com/injae/serdepp.
