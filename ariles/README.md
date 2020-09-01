Ariles
======

<table>
  <tr>
    <th>branch</th>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/head_2">HEAD v2</a>
    </td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/pkg_ros_2">pkg_ros</a><br/>
        (ROS package)
    </td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/pkg_freebsd">pkg_freebsd</a><br/>
        (FreeBSD package)
    </td>
  </tr>
  <tr>
    <th>CI status</th>
    <td align="center"><a href="https://travis-ci.org/asherikov/ariles"><img src="https://travis-ci.org/asherikov/ariles.svg?branch=head_2" alt="Build Status"></a></td>
    <td align="center"><a href="https://travis-ci.org/asherikov/ariles"><img src="https://travis-ci.org/asherikov/ariles.svg?branch=pkg_ros_2" alt="Build Status"></a></td>
    <td align="center">X</td>
  </tr>
</table>


Contents
========
* [Links](#links)
* [Introduction](#intro)
    * [Use cases](#uses)
* [Minimal example](#example)
* [Supported formats](#formats)
* [Supported types](#types)
* [Dependencies and compilation](#compilation)
* [Related software](#related)


<a name="links"></a>
Links
=====
* Documentation (Doxygen): https://asherikov.github.io/ariles/2/
* GitHub: https://github.com/asherikov/ariles
* Travis CI: https://travis-ci.org/asherikov/ariles
* Legacy 1.x.x version: https://github.com/asherikov/ariles/tree/head_1


<a name="intro"></a>
Introduction
============

`ariles` is a C++ meta-programming (reflection?) library with focus on
serialization and configuration. It employs preprocessor macro to automatically
generate boilerplate code which facilitates implementation of generic
processors (visitors) for classes. A number of visitors is included in the
library, in particular parsers and emitters of data in various formats, such as
`YAML`, `JSON`, `XML`, `ROS` parameter server. `ariles` also provides
predefined serialization wrappers for some common types, e.g., standard
containers, smart pointers, `Eigen` matrices.


<a name="uses"></a>
Use cases
---------

1. Parsing and generation of configuration files. Unlike some common
   serialization libraries, e.g., `boost::serialization`, `ariles` tries to be
   flexible while parsing by:
    - silently ignoring unused entries (if possible),
    - ignoring ordering of entries (if possible),
    - not discriminating attributes from childs in XML,
    - optionally ignoring missing entries.

2. Conversion between different formats, for example, `YAML` <-> `ROS`
   parameter server. Note that the conversion is not data-agnostic, i.e., the
   complete structure of the data must be represented in C++ code.

3. Collection of time-series data by flattening a class hierarchy to a list of
   name-value pairs (string-double).

4. `ariles` can emit `Octave` script code containing all data, which is useful
   for debugging of numerical software.

5. Due to flexibile parsing, `ariles` can be used to process generic file
   formats, e.g., `URDF`.



<a name="example"></a>
Minimal example
===============

Demo: https://asherikov.github.io/ariles/2/DEMO.html [`./tests/api_v2/demo_api_v2.cpp`]


Class [`./tests/api_v2/types/minimal.h`]:
```
class Configurable : public ariles2::DefaultBase
{
    #define ARILES2_ENTRIES(v) \
        ARILES2_TYPED_ENTRY(v, integer_member, int)
    #include ARILES2_INITIALIZE
};
```

Serialization:
```
Configurable configurable;
ariles2::apply<ariles2::yaml_cpp::Writer>("config.yaml", configurable);
ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, configurable);
```

Result:
```
ConfigurableEntryName:
    integer_member: 0
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
ariles2::apply<ariles2::ros::Writer>(nh, configurable, "/some_namespace/");
```



<a name="formats"></a>
Supported data representation formats
=====================================

`ariles` includes a number of optional modules that allow to work with
specific data representation formats, for example:

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

* A set of key-value pairs, output only, no dependencies:
  https://asherikov.github.io/ariles/2/group__array.html

* `graphviz` dot files for diagram generation:
  https://asherikov.github.io/ariles/2/group__graphviz.html


The complete list of modules is available at
https://asherikov.github.io/ariles/2/modules.html



<a name="types"></a>
Supported data types
====================

* Fundametal types: signed/unsigned integers, floats, booleans.
* Some STL classes (WIP): `std::string`, `std::vector`, `std::map`, `std::pair`, `std::shared_ptr`, `std::unique_ptr`.
* `Eigen` types: matrices, transforms, quaternions.
* `Boost` classes: `boost::optional`, `boost::movelib::unique_ptr`. `boost::shared_ptr`.
* Better enums -> https://github.com/aantron/better-enums.



<a name="compilation"></a>
Dependencies and compilation
============================

Dependencies
------------

`ariles` does not depend on new C++ features and is C++03 compliant.

Support of any data format, and corresponding dependency, can be enabled or
disabled via cmake options. The same applies to data types which depend on
external libraries. The only mandatory requirement is `Boost`.


Compilation with catkin
-----------------------

An example catkin package is provided in `pkg_ros` branch of the main
repository -> https://github.com/asherikov/ariles/tree/pkg_ros.



<a name="related"></a>
Related software
================

* https://github.com/PickNikRobotics/rosparam_shortcuts: a set of wrapper
  functions to read individual parameters from ROS parameter server. This tool
  serves pretty much the same purpose as `ariles2::ros::Reader`, but its
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

