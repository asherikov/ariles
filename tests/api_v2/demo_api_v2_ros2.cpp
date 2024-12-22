/**
    @file
    @author  Alexander Sherikov

    @copyright 2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/
// cppcheck-suppress-file duplInheritedMember


// ============================================================================
// HEADER INCLUSION
// ============================================================================

// `visitor` is an Ariles component which provides integration with a particular
// 3rd party library.
#include <ariles2/visitors/yaml_cpp.h>
#include <ariles2/visitors/ros2param.h>

// `adapter` is an Ariles component which adds support for serialization of
// certain type(s), e.g. Eigen types or Boost pointers.
#include <ariles2/adapters/basic.h>
#include <ariles2/adapters/eigen.h>
#include <ariles2/adapters/std_vector.h>
#include <ariles2/adapters/pointer.h>
#include <ariles2/ariles.h>



// ===============================================================
// DEFINING TYPES
// ===============================================================
namespace
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
        virtual ~ArilesBaseClass() = default;  // added to suppress compiler warnings

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


    class MyClass : public ArilesBaseClass,  // no need to inherit from ariles2::DefaultBase directly.
                    public NonArilesBaseClass
    {
// Declare entries, in this case we indicate inheritance from another
// Ariles class (ArilesBaseClass) and a member from a non-Ariles class
// (NonArilesBaseClass)
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_PARENT(v, ArilesBaseClass)                                                                                 \
    ARILES2_ENTRY_(v, eigen_vector)
        // Here ^ Ariles should not declare the inherited member, therefore we
        // use 'ARILES2_ENTRY_' instead of 'ARILES2_TYPED_ENTRY_'.

#include ARILES2_INITIALIZE


    public:
        ~MyClass() override = default;  // added to suppress compiler warnings


        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            // If you use your own method to initialize member variables,
            // it is up to you to properly initialize all entries and
            // parent classes.
            // all parents at once
            arilesVisitParents(visitor, param);
            // or one by one (either option is sufficient)
            ArilesBaseClass::arilesVisit(visitor, param);

            // custom default values for some members
            real_member = 100.0;
            eigen_vector_.setZero();
        }
    };


    class MyContainerClass : public ariles2::DefaultBase
    {
        // Some of the standard containers can be used with Ariles types.
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, my_class_vector, std::vector<MyClass>)                                                     \
    ARILES2_TYPED_ENTRY_(v, ptr, std::shared_ptr<MyClass>)
#include ARILES2_INITIALIZE

    public:
        virtual ~MyContainerClass() = default;
    };
}  // namespace


// ===============================================================
// SERIALIZATION & DESERIALIZATION
// ===============================================================

#include <iostream>  // std::cout


// run with "demo_api_v2_ros2 --ros-args --params-file /demo_api_v2_ros2.yaml"
int main(int argc, char *argv[])
{
    try
    {
        rclcpp::init(argc, argv);
        const rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared(
                "demo_api_v2_ros2",
                // although ariles provides Declarator visitor, declaring
                // parameters in general case is not possible since generic
                // arrays are stored as maps with indices used as keys: size of
                // such array is not know in advance so it is not possible to
                // declare all the necessary indices
                rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(
                        true));

        // read parameters loaded from file (demo_api_v2_ros2.yaml)
        {
            MyContainerClass my_container_class;

            // access members as usual
            my_container_class.my_class_vector_.emplace_back();
            ariles2::apply<ariles2::Defaults>(my_container_class.my_class_vector_[0]);

            /*
             * output
            my_class_vector:
              - real_member: 100
                integer_member: 12
                eigen_vector: [0, 0, 0]
            ptr:
              is_null: true
            */
            ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, my_container_class);

            // see demo_api_v2_ros2.yaml
            ariles2::apply<ariles2::ros2param::Reader>(nh->get_node_parameters_interface(), my_container_class);

            /*
             * output
            my_class_vector:
              - real_member: 100
                integer_member: 12
                eigen_vector: [0, 0, 0]
              - real_member: 110
                integer_member: 1
                eigen_vector: [0, 1, 0]
              - real_member: 111
                integer_member: 21
                eigen_vector: [1, 1, 0]
            ptr:
              is_null: true
            */
            ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, my_container_class);
        }

        // writing & reading
        {
            MyContainerClass my_container_class;
            my_container_class.ptr_ = std::make_shared<MyClass>();
            ariles2::apply<ariles2::Defaults>(*my_container_class.ptr_);

            my_container_class.my_class_vector_.emplace_back();
            ariles2::apply<ariles2::Defaults>(my_container_class.my_class_vector_[0]);
            my_container_class.my_class_vector_[0].real_member = 200;

            ariles2::apply<ariles2::ros2param::Writer>(nh->get_node_parameters_interface(), my_container_class);
            ariles2::apply<ariles2::ros2param::Reader>(nh->get_node_parameters_interface(), my_container_class);

            /*
             * output:
             *  - note that parameters loaded from file are preserved
             *  - it would be nice to drop old parameters, but it is impossible to undeclare them
            my_class_vector:
              - real_member: 200            # overridden
                integer_member: 12
                eigen_vector: [0, 0, 0]
              - real_member: 110
                integer_member: 1
                eigen_vector: [0, 1, 0]
              - real_member: 111
                integer_member: 21
                eigen_vector: [1, 1, 0]
            ptr:
              is_null: false                # overridden
              value:                        # added
                real_member: 100
                integer_member: 12
                eigen_vector: [0, 0, 0]
            */
            ariles2::apply<ariles2::yaml_cpp::Writer>(std::cout, my_container_class);
        }

        // declaring
        // - hardly useful, is going to declare my_class_vector_ to have only one member
        // - declarator does not read parameters into an Ariles class, a
        // separate pass with Reader visitor is needed
        {
            MyContainerClass my_container_class;
            my_container_class.my_class_vector_.emplace_back();
            ariles2::apply<ariles2::Defaults>(my_container_class.my_class_vector_[0]);
            ariles2::apply<ariles2::ros2param::Declarator>(nh->get_node_parameters_interface(), my_container_class);
        }

        // missing parameters
        //  - an attempt to read missing parameters results in an exception
        {
            MyClass my_class;
            try
            {
                ariles2::apply<ariles2::ros2param::Reader>(nh->get_node_parameters_interface(), my_class);
            }
            catch (const std::exception &e)
            {
                // "... Configuration file does not contain entry 'real_member'."
                std::cout << e.what() << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
        return (EXIT_FAILURE);
    }

    return (EXIT_SUCCESS);
}
