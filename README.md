Ariles
======

<table>
  <tr>
    <th>branch</th>
    <td align="center"><a href="https://github.com/asherikov/ariles/">master</a></td>
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
    <td align="center"><a href="https://travis-ci.org/asherikov/ariles"><img src="https://travis-ci.org/asherikov/ariles.svg?branch=head_1" alt="Build Status"></a></td>
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
    * [APIv1](#apiv1)
    * [APIv2](#apiv2)
* [Supported formats](#formats)
* [Supported types](#types)
* [Dependencies and compilation](#compilation)


<a name="links"></a>
Links
=====
* Documentation (Doxygen): https://asherikov.github.io/ariles/1/
* GitHub: https://github.com/asherikov/ariles
* Travis CI: https://travis-ci.org/asherikov/ariles


<a name="intro"></a>
Introduction
============

**Note: this is a legacy branch, the main development branch is https://github.com/asherikov/ariles/tree/head_2**

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

Currently `ariles` provides two API versions:
- APIv1: to be deprecated in the next major release,
- APIv2: new, unstable API.


<a name="apiv1"></a>
APIv1
-----

Demo: https://asherikov.github.io/ariles/1/DEMOv1.html [`./tests/api_v1/demo_api_v1.cpp`]


Class [`./tests/api_v1/types/minimal.h`]:
```
class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableEntryName"
    #define ARILES_AUTO_DEFAULTS
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY(integer_member, int)
    #include ARILES_INITIALIZE
};
```

Serialization:
```
Configurable configurable;
configurable.writeConfig<ariles::yaml_cpp>("config_file.yaml");
configurable.writeConfig<ariles::yaml_cpp>(std::cout);
```

Result:
```
ConfigurableEntryName:
    integer_member: 0
```

Deserialization:
```
configurable.readConfig<ariles::yaml_cpp>("config_file.yaml");
```

Conversion:
```
configurable.readConfig<ariles::yaml_cpp>("config_file.yaml");
configurable.writeConfig<ariles::ros>(nh, "/some_namespace/");
```


<a name="apiv2"></a>
APIv2
-----

Demo: https://asherikov.github.io/ariles/1/DEMOv2.html [`./tests/api_v2/demo_api_v2.cpp`]


Class [`./tests/api_v2/types/minimal.h`]:
```
class Configurable : public ariles::DefaultBase
{
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY(integer_member, int)
    #include ARILES_INITIALIZE
};
```

Serialization:
```
Configurable configurable;
ariles::apply<ariles::yaml_cpp::Writer>("config.yaml", configurable);
ariles::apply<ariles::yaml_cpp::Writer>(std::cout, configurable);
```

Result:
```
ConfigurableEntryName:
    integer_member: 0
```

Deserialization:
```
ariles::apply<ariles::yaml_cpp::Reader>("config.yaml", configurable);
```

Conversion:
```
ariles::apply<ariles::yaml_cpp::Reader>("config.yaml", configurable);
ariles::apply<ariles::ros::Writer>(nh, configurable, "/some_namespace/");
```


<a name="formats"></a>
Supported data formats
======================

Currently supported formats are (all are optional):

* `YAML` via `yaml-cpp`, both old C++03 and new API supported.
    - `yaml-cpp` does not comply with the specification when it emits NaN's and
      infinities, see https://github.com/jbeder/yaml-cpp/issues/507. `ariles`
      includes a workaround for this issue.

* `msgpack` via `msgpack-c`.

* `JSON` via `RapidJSON`, with optional Jsonnet (https://jsonnet.org/)
  preprocessing:
    - NaN's and infinities, which are not allowed by `JSON` specification, are
      optionally parsed / emitted using `boost::lexical_cast`.

* `XML` via `PugiXML`:
    - Attributes are treated as childs while parsing and are never used for
      emission.

* `Octave` script, output only, no dependencies:
    - Eigen matrices are written in the 'native' format, so they can be used
      directly, no reshaping is necessary.

    - Matlab might be supported, but has not been tested.

* `ROS` parameter server, via standard `ROS` libs.

* A set of key-value pairs, output only, no dependencies:
    - A vector of string-double pairs with flattened member names, e.g.,
      `<class.member_class.member, value>`.



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
