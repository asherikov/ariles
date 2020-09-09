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

// `visitor` is an Ariles component which provides integration with a particular
// 3rd party library.
#include <ariles2/visitors/yaml_cpp.h>
#include <ariles2/visitors/rosparam.h>
#include <ariles2/visitors/octave.h>

// `adapter` is an Ariles component which adds support for serialization of
// certain type(s), e.g. Eigen types or Boost pointers.
#include <ariles2/adapters/basic.h>
#include <ariles2/adapters/eigen.h>
#include <ariles2/adapters/std_vector.h>
#include <ariles2/ariles.h>



// ===============================================================
// DEFINING TYPES
// ===============================================================
namespace demo
{
    class ArilesBaseClass
      // must inherit from ariles2::DefaultBase
      : public ariles2::DefaultBase
    {
// Declare entries, in this case two numbers
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY(v, real_member, double)                                                                        \
    ARILES2_TYPED_ENTRY_(v, integer_member, int)
//         underscore ^ indicates that the name of the entry must be
// 'integer_member_' instead of 'integer_member', this is useful if your
// naming convention requires trailing underscores for member variables.

// Initialize ariles
#include ARILES2_INITIALIZE

    public:
        virtual ~ArilesBaseClass(){};  // added to suppress compiler warnings

        // This method is called every time you deserialize a class. If
        // omitted, the default automatically generated method is used.
        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
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
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, ArilesBaseClass)                                                                                 \
    ARILES2_ENTRY_(v, eigen_vector)
        //              In this case ^ Ariles should not declare the inherited
        // member, therefore we use 'ARILES2_ENTRY_' instead of 'ARILES2_TYPED_ENTRY_'.

#include ARILES2_INITIALIZE


    public:
        virtual ~MyClass(){};  // added to suppress compiler warnings


        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
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


    class MyContainerClass : public ariles2::DefaultBase
    {
        // Some of the standard containers can be used with Ariles types.
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, my_class_vector, std::vector<MyClass>)
#include ARILES2_INITIALIZE
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
    my_container_class.my_class_vector_.size();
    my_container_class.my_class_vector_.push_back(demo::MyClass());
    ariles2::apply<ariles2::Defaults>(my_container_class.my_class_vector_[0]);


    // YAML
    /*
     * When you serialize `my_container_class` to YAML you get the following:
     * -----
        MyContainerClass:
          my_class_vector:
            - real_member: 100
              integer_member: 12
              eigen_vector: [0, 0, 0]
     * -----
     * Note that the trailing underscores are omitted for all members. This
     * applies to all supported representations.
     */
    {
        // You can read and write YAML configuration files as follows:
        ariles2::apply<ariles2::yaml_cpp::Writer>("config.yaml", my_container_class);
        ariles2::apply<ariles2::yaml_cpp::Reader>("config.yaml", my_container_class);

        // Sometimes it may be useful to dump configuration to std::cout
        ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, my_container_class);

        // In some situations it is more convenient to instantiate Reader and
        // Writer classes explicitly, e.g., if you keep configurations of several
        // classes in the same file
        ariles2::yaml_cpp::Writer writer("config.yaml");
        ariles2::apply(writer, my_container_class);

        ariles2::yaml_cpp::Reader reader("config.yaml");
        ariles2::apply(reader, my_container_class);
    }


    // ROS parameter server
    {
        ros::NodeHandle nh;

        // read/write
        ariles2::apply<ariles2::rosparam::Writer>(nh, my_container_class);
        ariles2::apply<ariles2::rosparam::Reader>(nh, my_container_class);
        // parameters can be uploaded to parameter server in advance using
        // roslaunch, see http://wiki.ros.org/roslaunch/XML/rosparam

        // read/write with namespace
        ariles2::apply<ariles2::rosparam::Writer>(nh, my_container_class, "/some_namespace/");
        ariles2::apply<ariles2::rosparam::Reader>(nh, my_container_class, "/some_namespace/");

        // Reader / Writer classes
        ariles2::rosparam::Writer writer(nh);
        ariles2::apply(writer, my_container_class);

        ariles2::rosparam::Reader reader(nh);
        ariles2::apply(reader, my_container_class);
    }


    // Octave
    {
        // Octave visitor supports only writing
        ariles2::apply<ariles2::octave::Writer>("debug.m", my_container_class);
        // the generated file can later be loaded in Octave with
        // 'source debug.m' for debugging
    }


    return (0);
}
