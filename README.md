Ariles is a C++ serialization/configuration library with some extra automatic
code generation capabilities. The main features are

* Ariles relies on third-party libraries for support of various data
  representation formats. Currently supported formats are (all are optional):
  - yaml (via yaml-cpp, both old C++03 and new API supported);
  - msgpack (via msgpack-c);
  - ROS parameter server (via standard ROS libs).

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
configurable.readConfig<ariles::yaml::Reader>("config_file.yaml");
```
