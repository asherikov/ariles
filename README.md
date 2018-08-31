[![Build Status](https://travis-ci.org/asherikov/ariles.svg?branch=master)](https://travis-ci.org/asherikov/ariles)


Introduction
============

Ariles is a C++ serialization/configuration library with some extra automatic
code generation capabilities. It relies on third-party libraries for support of
various data representation formats. The library also provides some predefined
serialization wrappers for common types as described below.


Supported formats
=================

Currently supported formats are (all are optional):

* yaml via yaml-cpp, both old C++03 and new API supported;
* msgpack via msgpack-c;
* JSON via RapidJSON, with optional Jsonnet (https://jsonnet.org/) preprocessing;
* Octave script, output only, no dependencies;
* ROS parameter server, via standard ROS libs.


Supported types
===============

* Some STL containers (WIP).
* Eigen matrices.
* Boost pointers.
* boost::optional.
* Better enums -> https://github.com/aantron/better-enums.


Features
========

* Ariles, unlike many C++ serialization libraries, primarily relies on
  preprocessor (similar ideas are described at
  http://cplusplus.bordoon.com/dark_side.html). This allows to
  - use standard virtual method mechanism for serialization via base classes,
    which is not possible when templated serialization methods are used (as,
    e.g., in boost serialization);
  - avoid code repetition and related mistakes, for example, the following code
    declares an integer variable and marks it for serialization with a single
    directive:
```
class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "Configurable"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY(integer_member, int)
    #include ARILES_INITIALIZE


    public:
        virtual void setDefaults()
        {
            integer_member = 10;
        }
};
```

* Ariles generates utility methods to reduce amount of code necessary for
  serialization / deserialization, e.g., the following initializes a class from
  a YAML file (explicit instantiation of an 'archive' class is not needed):
```
Configurable configurable;
configurable.readConfig<ariles::yaml_cpp>("config_file.yaml");
```


Notes
=====

Octave
------

* Ariles outputs a serialized class to an Octave script file, which can later
  be `source`d from Octave to create a struct representation of the class.

* Eigen matrices are written in the 'native' format, so they can be used
  directly, no reshaping is necessary.

* Matlab might be supported, but has not been tested.


yaml-cpp
--------

* yaml-cpp does not comply with the specification when it emits NaN's and
  infinities, see https://github.com/jbeder/yaml-cpp/issues/507. Ariles
  includes a workaround for this issue.


RapidJSON
---------

* NaN's and infinities are enabled with corresponding flags, which is not
  allowed by JSON specification, see https://github.com/Tencent/rapidjson/issues/972.
