/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


// ============================================================================
// HEADER INCLUSION
// ============================================================================

/*
 * Version I: selective inclusion
 * ------------------------------
 */
#include <ariles/internal/build_config.h>

// `bridge` is an Ariles component which provides integration with a particular
// 3rd party library.
#include "ariles/bridges/yaml_cpp.h"
#include "ariles/bridges/ros.h"
#include "ariles/bridges/octave.h"

// `adapter` is an Ariles component which adds support for serialization of
// certain type(s), e.g. Eigen types or Boost pointers.
#include "ariles/adapters_all.h"
#include "ariles/ariles.h"


/*
 * Version II: complete inclusion
 * ------------------------------
 */
// `ariles_all.h` header includes all bridges and adapters detected at compile
// time, it may not be generated in some configurations.
//#include "ariles/ariles_all.h"



// ===============================================================
// DEFINING TYPES
// ===============================================================
namespace demo
{
    class ArilesBaseClass
        // must inherit from ariles::ConfigurableBase
        : public ariles::ConfigurableBase
    {
        // Unique entry name, to be safe use only alphanumeric characters and underscores
        #define ARILES_SECTION_ID "ArilesBaseClass"

        // Declare entries, in this case two numbers
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY(real_member, double) \
            ARILES_TYPED_ENTRY_(integer_member, int)
        //         underscore ^ indicates that the name of the entry must be
        // 'integer_member_' instead of 'integer_member', this is useful if your
        // naming convention requires trailining underscores for member variables.

        // Initialize ariles
        #include ARILES_INITIALIZE

        public:
            virtual ~ArilesBaseClass(){}; // added to suppress compiler warnings

            // setDefaults() is a method which is called every time you deserialize
            // a class. You can implement it manually as here, or request its
            // automatic generation using ARILES_AUTO_DEFAULTS as demonstrated for
            // other classes below.
            virtual void setDefaults()
            {
                real_member = 0.0;
                integer_member_ = 12;
            }
    };


    class NonArilesBaseClass
    {
        public:
            // Eigen types are supported too, see below
            Eigen::Vector3d eigen_vector_;
    };


    class MyClass
        :   public ArilesBaseClass, // no need to inherit from ConfigurableBase directly.
            public NonArilesBaseClass
    {
        #define ARILES_SECTION_ID "MyClass"

        // Declare entries, in this case we indicate inheritance from another
        // Ariles class (ArilesBaseClass) and a member from a non-Ariles class
        // (NonArilesBaseClass)
        #define ARILES_ENTRIES \
            ARILES_PARENT(ArilesBaseClass) \
            ARILES_ENTRY_(eigen_vector)
        //              In this case ^ Ariles should not declare the inherited
        // member, therefore we use 'ARILES_ENTRY_' instead of 'ARILES_TYPED_ENTRY_'.

        #include ARILES_INITIALIZE


        public:
            virtual ~MyClass(){}; // added to suppress compiler warnings

            virtual void setDefaults()
            {
                // If you implement setDefaults() manually, it is up to you to
                // properly initialize all entries and parent classes.
                ArilesBaseClass::setDefaults();

                // custom default values for some members
                real_member = 100.0;
                eigen_vector_.setZero();
            }
    };


    class MyContainerClass
        :   public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "MyContainerClass"

        #define ARILES_AUTO_DEFAULTS // Generate setDefaults() automatically

        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(myclass_vector, std::vector<MyClass>)
        //      Some of the standard containers ^^^^^^^^^^^^^^^^^^^^ can be used
        // with Ariles types.

        #include ARILES_INITIALIZE
    };
}


// ===============================================================
// SERIALIZATION & DESERIALIZATION
// ===============================================================

#include <iostream> // std::cout

int main()
{
    demo::MyContainerClass my_container_class;

    // access members as usual
    my_container_class.myclass_vector_.size();
    my_container_class.myclass_vector_.push_back(demo::MyClass());
    my_container_class.myclass_vector_[0].setDefaults();


    // YAML
    /*
     * When you serialize `my_container_class` to YAML you get the following:
     * -----
        MyContainerClass:
          myclass_vector:
            - real_member: 100
              integer_member: 12
              eigen_vector: [0, 0, 0]
     * -----
     * Note that the trailing underscores are omitted for all members. This
     * applies to all supported representations.
     */
    {
        // You can read and write YAML configuration files as follows:
        my_container_class.writeConfig<ariles::yaml_cpp>("config.yaml");
        my_container_class.readConfig<ariles::yaml_cpp>("config.yaml");

        // Sometimes it may be useful to dump configuration to std::cout
        my_container_class.writeConfig<ariles::yaml_cpp>(std::cout);

        // In some situations it is more convenient to instantiate Reader and
        // Writer classes explicitly, e.g., if you keep configurations of several
        // classses in the same file
        ariles::yaml_cpp::Writer writer("config.yaml");
        my_container_class.writeConfig(writer);

        ariles::yaml_cpp::Reader reader("config.yaml");
        my_container_class.readConfig(reader);
    }


    // ROS parameter server
    {
        ros::NodeHandle nh;

        // read/write
        my_container_class.writeConfig<ariles::ros>(nh);
        my_container_class.readConfig<ariles::ros>(nh);
        // parameters can be uploaded to parameter server in advance using
        // roslaunch, see http://wiki.ros.org/roslaunch/XML/rosparam

        // read/write with namespace
        my_container_class.writeConfig<ariles::ros>(nh, "/some_namespace/");
        my_container_class.readConfig<ariles::ros>(nh, "/some_namespace/");

        // Reader / Wrter classes
        ariles::ros::Writer writer(nh);
        my_container_class.writeConfig(writer);

        ariles::ros::Reader reader(nh);
        my_container_class.readConfig(reader);
    }


    // Octave
    {
        // Octave bridge supports only writing
        my_container_class.writeConfig<ariles::octave>("debug.m");
        // the generated file can later be loaded in Octave with
        // 'source debug.m' for debugging
    }


    return (0);
}
