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

// `visitor` is an Ariles component which provides integration with a particular
// 3rd party library.
#include "ariles/visitors/yaml_cpp.h"
#include "ariles/visitors/ros.h"
#include "ariles/visitors/octave.h"

// `adapter` is an Ariles component which adds support for serialization of
// certain type(s), e.g. Eigen types or Boost pointers.
#include "ariles/adapters/basic.h"
#include "ariles/adapters/eigen.h"
#include "ariles/adapters/vector.h"
#include "ariles/ariles2.h"



// ===============================================================
// DEFINING TYPES
// ===============================================================
namespace demo
{
    class ArilesBaseClass
      // must inherit from ariles::DefaultBase
      : public ariles::DefaultBase
    {
// Declare entries, in this case two numbers
#define ARILES_ENTRIES                                                                                                 \
    ARILES_TYPED_ENTRY(real_member, double)                                                                            \
    ARILES_TYPED_ENTRY_(integer_member, int)
//         underscore ^ indicates that the name of the entry must be
// 'integer_member_' instead of 'integer_member', this is useful if your
// naming convention requires trailining underscores for member variables.

// Initialize ariles
#include ARILES_INITIALIZE

    public:
        virtual ~ArilesBaseClass(){};  // added to suppress compiler warnings

        // This method is called every time you deserialize a class. If
        // omitted, the default automatically generated method is used.
        void arilesVisit(const ariles::Defaults & /*visitor*/, const ariles::Defaults::Parameters & /*param*/)
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


    class MyClass : public ArilesBaseClass,  // no need to inherit from ConfigurableBase directly.
                    public NonArilesBaseClass
    {
// Declare entries, in this case we indicate inheritance from another
// Ariles class (ArilesBaseClass) and a member from a non-Ariles class
// (NonArilesBaseClass)
#define ARILES_ENTRIES                                                                                                 \
    ARILES_PARENT(ArilesBaseClass)                                                                                     \
    ARILES_ENTRY_(eigen_vector)
        //              In this case ^ Ariles should not declare the inherited
        // member, therefore we use 'ARILES_ENTRY_' instead of 'ARILES_TYPED_ENTRY_'.

#include ARILES_INITIALIZE


    public:
        virtual ~MyClass(){};  // added to suppress compiler warnings


        void arilesVisit(const ariles::Defaults &visitor, const ariles::Defaults::Parameters &param)
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


    class MyContainerClass : public ariles::DefaultBase
    {
        // Some of the standard containers can be used with Ariles types.
#define ARILES_ENTRIES ARILES_TYPED_ENTRY_(myclass_vector, std::vector<MyClass>)
#include ARILES_INITIALIZE
    };
}  // namespace demo


// ===============================================================
// SERIALIZATION & DESERIALIZATION
// ===============================================================

#include <iostream>  // std::cout

int main()
{
    demo::MyContainerClass my_container_class;

    // access members as usual
    my_container_class.myclass_vector_.size();
    my_container_class.myclass_vector_.push_back(demo::MyClass());
    ariles::apply<ariles::Defaults>(my_container_class.myclass_vector_[0]);


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
        ariles::apply<ariles::yaml_cpp::Writer>("config.yaml", my_container_class);
        ariles::apply<ariles::yaml_cpp::Reader>("config.yaml", my_container_class);

        // Sometimes it may be useful to dump configuration to std::cout
        ariles::apply<ariles::yaml_cpp::Writer>(std::cout, my_container_class);

        // In some situations it is more convenient to instantiate Reader and
        // Writer classes explicitly, e.g., if you keep configurations of several
        // classses in the same file
        ariles::yaml_cpp::Writer writer("config.yaml");
        ariles::apply(writer, my_container_class);

        ariles::yaml_cpp::Reader reader("config.yaml");
        ariles::apply(reader, my_container_class);
    }


    // ROS parameter server
    {
        ros::NodeHandle nh;

        // read/write
        ariles::apply<ariles::ros::Writer>(nh, my_container_class);
        ariles::apply<ariles::ros::Reader>(nh, my_container_class);
        // parameters can be uploaded to parameter server in advance using
        // roslaunch, see http://wiki.ros.org/roslaunch/XML/rosparam

        // read/write with namespace
        ariles::apply<ariles::ros::Writer>(nh, my_container_class, "/some_namespace/");
        ariles::apply<ariles::ros::Reader>(nh, my_container_class, "/some_namespace/");

        // Reader / Wrter classes
        ariles::ros::Writer writer(nh);
        ariles::apply(writer, my_container_class);

        ariles::ros::Reader reader(nh);
        ariles::apply(reader, my_container_class);
    }


    // Octave
    {
        // Octave visitor supports only writing
        ariles::apply<ariles::octave::Writer>("debug.m", my_container_class);
        // the generated file can later be loaded in Octave with
        // 'source debug.m' for debugging
    }


    return (0);
}
