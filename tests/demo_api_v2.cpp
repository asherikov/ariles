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

#define ARILES_API_VERSION 2

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
#include "ariles/ariles2.h"



// ===============================================================
// DEFINING TYPES
// ===============================================================
namespace demo
{
    class ArilesBaseClass
        // must inherit from ariles::Base
        : public ariles::Base
    {
        // Unique entry name, to be safe use only alphanumeric characters and underscores
        #define ARILES_DEFAULT_ID "ArilesBaseClass"

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

            // This method is called every time you deserialize a class. If
            // omitted, the default automatically generated method is used.
            void arilesVisit(   const ariles::defaults::Visitor &/*visitor*/,
                                const ariles::defaults::Visitor::Parameters &/*param*/)
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
        #define ARILES_DEFAULT_ID "MyClass"

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


            void arilesVisit(   const ariles::defaults::Visitor & visitor,
                                const ariles::defaults::Visitor::Parameters & param)
            {
                // If you use your own method to initialize member variables,
                // it is up to you to properly initialize all entries and
                // parent classes.
                ArilesBaseClass::arilesVisit(visitor, param);

                // custom default values for some members
                real_member = 100.0;
                eigen_vector_.setZero();
            }
    };


    class MyContainerClass
        :   public ariles::Base
    {
        #define ARILES_DEFAULT_ID "MyContainerClass"

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
    my_container_class.myclass_vector_[0].ariles<ariles::defaults::Visitor>();


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
        my_container_class.ariles<ariles::yaml_cpp::Writer>("config.yaml");
        my_container_class.ariles<ariles::yaml_cpp::Reader>("config.yaml");

        // Sometimes it may be useful to dump configuration to std::cout
        my_container_class.ariles<ariles::yaml_cpp::Writer>(std::cout);

        // In some situations it is more convenient to instantiate Reader and
        // Writer classes explicitly, e.g., if you keep configurations of several
        // classses in the same file
        ariles::yaml_cpp::Writer writer("config.yaml");
        my_container_class.ariles(writer);

        ariles::yaml_cpp::Reader reader("config.yaml");
        my_container_class.ariles(reader);
    }


    // ROS parameter server
    {
        ros::NodeHandle nh;

        // read/write
        my_container_class.ariles<ariles::ros::Writer>(nh);
        my_container_class.ariles<ariles::ros::Reader>(nh);
        // parameters can be uploaded to parameter server in advance using
        // roslaunch, see http://wiki.ros.org/roslaunch/XML/rosparam

        // read/write with namespace
        my_container_class.ariles<ariles::ros::Writer>(nh, "/some_namespace/");
        my_container_class.ariles<ariles::ros::Reader>(nh, "/some_namespace/");

        // Reader / Wrter classes
        ariles::ros::Writer writer(nh);
        my_container_class.ariles(writer);

        ariles::ros::Reader reader(nh);
        my_container_class.ariles(reader);
    }


    // Octave
    {
        // Octave bridge supports only writing
        my_container_class.ariles<ariles::octave::Writer>("debug.m");
        // the generated file can later be loaded in Octave with
        // 'source debug.m' for debugging
    }


    return (0);
}
