Migration from version 1.X to 2.X
=================================

APIs of versions 1.X and 2.X of `ariles` are not compatible with each other,
but are conceptually similar, this document highlights differences and gives
some hints regarding migration of existing code to the new API. Note that
packages of versions 1.X and 2.X are compatible and can be used simultaneously.


Declaration & defaults
----------------------

Old API example:
```
    class ArilesBaseClass : public ariles::ConfigurableBase
    {
#define ARILES_SECTION_ID "ArilesBaseClass"
#define ARILES_ENTRIES                          \
    ARILES_TYPED_ENTRY(real_member, double)     \
    ARILES_TYPED_ENTRY_(integer_member, int)
#include ARILES_INITIALIZE

    public:
        virtual void setDefaults()
        {
            real_member = 0.0;
            integer_member_ = 12;
        }
    };
```

New API:
```
// 1. namespace has been renamed: ariles -> ariles2
// 2. classes should inherit from ariles2::DefaultBase
    class ArilesBaseClass : public ariles2::DefaultBase
    {
// 3. ARILES_SECTION_ID has been renamed to ARILES2_DEFAULT_ID and is now
// optional -- the default ID is now an empty string.
// 4. ARILES_ prefix has been replaced with ARILES2_
// 5. all entries now should pass an auxiliary parameter 'v' in this example,
// but the name can be arbitrary
#define ARILES2_ENTRIES(v)                          \
    ARILES2_TYPED_ENTRY(v, real_member, double)     \
    ARILES2_TYPED_ENTRY_(v, integer_member, int)
#include ARILES2_INITIALIZE

    public:
// 6. a method that initializes fields to their defaults is now always
// generated automatically - it is not necessary to specify
// ARILES_AUTO_DEFAULTS
// 7. the following method can be used to override automatically
// generated initialization method, note that it is not virtual.
        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            real_member = 0.0;
            integer_member_ = 12;
        }
    };
```


Operations on ariles classes
----------------------------

Old API:
```
MyArilesClass my_ariles_class;
my_container_class.setDefaults();

my_ariles_class.writeConfig<ariles::yaml_cpp>("config.yaml");
my_ariles_class.readConfig<ariles::yaml_cpp>("config.yaml");

ariles::ros::Writer writer(nh);
my_ariles_class.writeConfig(writer);

ariles::ros::Reader reader(nh);
my_ariles_class.readConfig(reader);
```

New API:
```
MyArilesClass my_ariles_class;

// 1. all operations on ariles classes are now performed using
// 'ariles2::apply()' function instead of class member methods

ariles2::apply<ariles2::Defaults>();

ariles2::apply<ariles2::yaml_cpp::Writer>("config.yaml", my_ariles_class);
ariles2::apply<ariles2::yaml_cpp::Reader>("config.yaml", my_ariles_class);

ariles2::rosparam::Writer writer(nh);
ariles2::apply(writer, my_ariles_class);

ariles2::rosparam::Reader reader(nh);
ariles2::apply(reader, my_ariles_class);
```
