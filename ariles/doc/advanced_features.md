Advanced features
=================

* 'Sloppy' `std::map` and `std::pair`: maps and pairs where the key or the
  first element is an `std::string` can optionally be saved / loaded as maps if
  supported by the data format. For example:
```
    std_map:
        key: value
```
  instead of
```
    std_map:
        first: key
        second: value
```

* 'Polymorphic' configurations: `ariles::Any` class defined in `ariles/types.h`
  provides functionality similar to `protobuf::Any` -- this class automatically
  instantiates and reads configuration of some user-defined class based on its
  string id and stores a pointer to its base class. See
  `tests/api_v(1|2)/types/any.h` for an example.

