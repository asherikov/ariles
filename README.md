* Doxygen: https://asherikov.github.io/ariles/
* Travis CI: https://travis-ci.org/asherikov/ariles [![Build Status](https://travis-ci.org/asherikov/ariles.svg?branch=master)](https://travis-ci.org/asherikov/ariles)


Introduction
============

Ariles is a C++ serialization/configuration library with some extra automatic
code generation capabilities. It relies on third-party libraries for support of
various data representation formats. The library also provides some predefined
serialization wrappers for common types as described below.

Ariles was not designed with an intention to address certain limitations of
other serialization libraries such as https://uscilab.github.io/cereal/ or
https://www.boost.org/doc/libs/1_67_0/libs/serialization/doc/, but rather
accidentally evolved from a bunch YAML serialization macro. Ariles, however,
differs from the common C++ serialization libs in several aspects:

* It is intended to be used for both serialization and configuration,
  therefore, it assumes that the input files could be written by a human or
  generated by another program. For this reason, Ariles tries to be flexible
  while parsing, e.g., for most formats it silently ignores unused entries,
  ignores ordering of entries, does not discriminate attributes from childs in
  XML, can optionally ignore missing entries, etc.

* Ariles is meant to be used primarily for 'invasive' serialization, so it
  generates class methods for serialization, initialization to default values,
  and postprocessing after reading data from a file. Also, Ariles relies on
  virtual methods for serialization via base classes (unlike boost and cereal).

* Ariles supports some 'exotic' serialization formats, such as ROS parameter
  server and Octave script.


Supported formats
=================

Currently supported formats are (all are optional):

* YAML via yaml-cpp, both old C++03 and new API supported.
    - yaml-cpp does not comply with the specification when it emits NaN's and
      infinities, see https://github.com/jbeder/yaml-cpp/issues/507. Ariles
      includes a workaround for this issue.

* msgpack via msgpack-c.

* JSON via RapidJSON, with optional Jsonnet (https://jsonnet.org/)
  preprocessing:
    * NaN's and infinities are enabled with corresponding flags, which is not
      allowed by JSON specification, see
      https://github.com/Tencent/rapidjson/issues/972.

* XML via PugiXML:
    - Attributes are treated as childs while reading and are never used for
      writing.

* Octave script, output only, no dependencies:
    - Ariles outputs a serialized class to an Octave script file, which can
      later be `source`d from Octave to create a struct representation of the
      class.

    - Eigen matrices are written in the 'native' format, so they can be used
      directly, no reshaping is necessary.

    - Matlab might be supported, but has not been tested.

* ROS parameter server, via standard ROS libs.


Supported types
===============

* Some STL containers (WIP).
* Eigen matrices.
* Boost pointers.
* boost::optional.
* Better enums -> https://github.com/aantron/better-enums.



Minimal example
===============
Class:
```
class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "ConfigurableEntryName"
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY(integer_member, int)
    #include ARILES_INITIALIZE
};
```

Serialization:
```
Configurable configurable;
configurable.writeConfig<ariles::yaml_cpp>("config_file.yaml");
```

Result:
```
ConfigurableEntryName:
    integer_member: 0
```

Deserialization:
```
Configurable configurable;
configurable.readConfig<ariles::yaml_cpp>("config_file.yaml");
```
