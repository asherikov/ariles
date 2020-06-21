Ariles
======

<table>
  <tr>
    <th>branch</th>
    <td align="center"><a href="https://github.com/asherikov/ariles/tree/head_2">HEAD v2</a></td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/pkg_ros">pkg_ros</a>
        (<a href="https://index.ros.org/p/ariles_ros/">ROS package</a>)
    </td>
    <td align="center">
        <a href="https://github.com/asherikov/ariles/tree/pkg_freebsd">pkg_freebsd</a>
        (FreeBSD package)
    </td>
  </tr>
  <tr>
    <th>CI status</th>
    <td align="center"><a href="https://travis-ci.org/asherikov/ariles"><img src="https://travis-ci.org/asherikov/ariles.svg?branch=head_2" alt="Build Status"></a></td>
    <td align="center"><a href="https://travis-ci.org/asherikov/ariles"><img src="https://travis-ci.org/asherikov/ariles.svg?branch=pkg_ros" alt="Build Status"></a></td>
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


<a name="links"></a>
Links
=====
* Documentation (Doxygen): https://asherikov.github.io/ariles/2/
* GitHub: https://github.com/asherikov/ariles
* Travis CI: https://travis-ci.org/asherikov/ariles


<a name="intro"></a>
Introduction
============

`ariles` is a C++ reflection library with focus on serialization/configuration.
It relies on other open-source libraries for parsing and emission of data in
different formats, in particular: `YAML`, `JSON`, `XML`, `ROS` parameter
server. The library also provides some predefined serialization wrappers for
common types, e.g., some standard containers and smart pointers.


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

3. `ariles` facilitates collection of time-series data by flattening a class
   hierarchy to a list of name-value pairs (string-double).

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
ariles2::apply<ariles2::yaml_cpp::Reader>("config.yaml", configurable);
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
