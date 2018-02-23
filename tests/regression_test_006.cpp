/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include "utility.h"

// Enable ROS configuration files (must be first)
#include "ariles/format_ros.h"

#include "ariles/adapters_all.h"

// definition of ariles::ConfigurableBase
#include "ariles/ariles.h"


// ===============================================================
// TYPES
// ===============================================================

enum SomeEnum
{
    UNDEFINED = 0,
    SOME_VALUE = 1,
    ANOTHER_VALUE = 2
};


class Configurable : public ariles::ConfigurableBase
{
    #define ARILES_SECTION_ID "Configurable"
    #define ARILES_CONSTRUCTOR Configurable
    #define ARILES_ENTRIES \
        ARILES_TYPED_ENTRY_(integer,     int) \
        ARILES_TYPED_ENTRY_(real,        double) \
        ARILES_TYPED_ENTRY_(string,      std::string) \
        ARILES_TYPED_ENTRY_(vector,      Eigen::Vector3d) \
        ARILES_TYPED_ENTRY_(matrix,      Eigen::Matrix3d) \
        ARILES_TYPED_ENTRY_(matrix_x,    Eigen::MatrixXd) \
        ARILES_TYPED_ENTRY_(std_vector,          std::vector<double>) \
        ARILES_TYPED_ENTRY_(std_nested_vector,   std::vector< std::vector<double> >) \
        ARILES_TYPED_ENTRY_(std_vector_evector,  std::vector<Eigen::Vector3d>) \
        ARILES_TYPED_ENTRY_(std_nested_vector_evector, std::vector< std::vector<Eigen::Vector3d> >) \
        ARILES_TYPED_ENTRY_(enum, SomeEnum)
    #include ARILES_INITIALIZE


    public:
        Configurable()
        {
            setDefaults();
        }


        virtual void setDefaults()
        {
            integer_ = 10;
            real_ = 1.33;
            string_ = "blahblah";
            vector_.setConstant(3);
            matrix_ << 1, 2, 3, 4, 5, 6, 7, 8, 9;

            matrix_x_.resize(2, 3);
            matrix_x_ << 8, 7, 6, 3, 2, 1;

            std_vector_.resize(5);
            for(std::size_t i = 0; i < std_vector_.size(); ++i)
            {
                std_vector_[i] = i * 5.22 + 2.3;
            }

            std_nested_vector_.resize(3);
            for(std::size_t i = 0; i < std_nested_vector_.size(); ++i)
            {
                std_nested_vector_[i].resize(7-i);

                for(std::size_t j = 0; j < std_nested_vector_[i].size(); ++j)
                {
                    std_nested_vector_[i][j] = 5.2 + i*0.1 + j*0.3;
                }
            }

            std_vector_evector_.resize(4);
            for(std::size_t i = 0; i < std_vector_evector_.size(); ++i)
            {
                std_vector_evector_[i].setConstant(i*9 + 0.43);
            }

            std_nested_vector_evector_.resize(2);
            for(std::size_t i = 0; i < std_nested_vector_evector_.size(); ++i)
            {
                std_nested_vector_evector_[i].resize(1+i);

                for(std::size_t j = 0; j < std_nested_vector_evector_[i].size(); ++j)
                {
                    std_nested_vector_evector_[i][j].setConstant(5.2 + i*0.1 + j*0.3);
                }
            }

            enum_ = ANOTHER_VALUE;
        }


        void randomize()
        {
            integer_ = GET_RANDOM_INT;
            real_ = GET_RANDOM_REAL;
            string_ = "blahblah";
            vector_.setRandom();
            matrix_.setRandom();

            matrix_x_.resize(2, 3);
            matrix_x_.setRandom();

            std_vector_.resize(5);
            for(std::size_t i = 0; i < std_vector_.size(); ++i)
            {
                std_vector_[i] = GET_RANDOM_REAL;
            }

            std_nested_vector_.resize(3);
            for(std::size_t i = 0; i < std_nested_vector_.size(); ++i)
            {
                std_nested_vector_[i].resize(7-i);

                for(std::size_t j = 0; j < std_nested_vector_[i].size(); ++j)
                {
                    std_nested_vector_[i][j] = GET_RANDOM_REAL;
                }
            }

            std_vector_evector_.resize(4);
            for(std::size_t i = 0; i < std_vector_evector_.size(); ++i)
            {
                std_vector_evector_[i].setRandom();
            }

            std_nested_vector_evector_.resize(2);
            for(std::size_t i = 0; i < std_nested_vector_evector_.size(); ++i)
            {
                std_nested_vector_evector_[i].resize(1+i);

                for(std::size_t j = 0; j < std_nested_vector_evector_[i].size(); ++j)
                {
                    std_nested_vector_evector_[i][j].setRandom();
                }
            }

            enum_ = ANOTHER_VALUE;
            finalize();
        }
};


// ===============================================================
// FIXTURES
// ===============================================================


// comparison
template<class t_Configurable_out, class t_Configurable_in>
void    compare(const t_Configurable_out    &configurable_out,
                const t_Configurable_in     &configurable_in)
{
    BOOST_CHECK_EQUAL(configurable_out.integer_,          configurable_in.integer_);
    BOOST_CHECK_CLOSE(configurable_out.real_,             configurable_in.real_, g_tolerance);
    BOOST_CHECK(configurable_out.vector_.isApprox(configurable_in.vector_, g_tolerance));
    BOOST_CHECK(configurable_out.matrix_.isApprox(configurable_in.matrix_, g_tolerance));
    BOOST_CHECK(configurable_out.matrix_x_.isApprox(configurable_in.matrix_x_, g_tolerance));
    BOOST_CHECK_EQUAL(configurable_out.string_,           configurable_in.string_);

    BOOST_CHECK_EQUAL(configurable_out.std_vector_.size(),                configurable_in.std_vector_.size());
    BOOST_CHECK_EQUAL(configurable_out.std_nested_vector_.size(),         configurable_in.std_nested_vector_.size());
    BOOST_CHECK_EQUAL(configurable_out.std_vector_evector_.size(),        configurable_in.std_vector_evector_.size());
    BOOST_CHECK_EQUAL(configurable_out.std_nested_vector_evector_.size(), configurable_in.std_nested_vector_evector_.size());

    for (std::size_t i = 0; i < configurable_out.std_vector_.size(); ++i)
    {
        BOOST_CHECK_CLOSE(configurable_out.std_vector_[i],
                    configurable_in.std_vector_[i],
                    g_tolerance);
    }

    for (std::size_t i = 0; i < configurable_out.std_vector_evector_.size(); ++i)
    {
        BOOST_CHECK(configurable_out.std_vector_evector_[i].isApprox(
                    configurable_in.std_vector_evector_[i], g_tolerance));
    }


    for (std::size_t i = 0; i < configurable_out.std_nested_vector_.size(); ++i)
    {
        BOOST_CHECK_EQUAL(configurable_out.std_nested_vector_[i].size(),  configurable_in.std_nested_vector_[i].size());
        for (std::size_t j = 0; j < configurable_out.std_nested_vector_[i].size(); ++j)
        {
            BOOST_CHECK_CLOSE(configurable_out.std_nested_vector_[i][j],
                        configurable_in.std_nested_vector_[i][j],
                        g_tolerance);
        }
    }

    for (std::size_t i = 0; i < configurable_out.std_nested_vector_evector_.size(); ++i)
    {
        BOOST_CHECK_EQUAL(configurable_out.std_nested_vector_evector_[i].size(),  configurable_in.std_nested_vector_evector_[i].size());
        for (std::size_t j = 0; j < configurable_out.std_nested_vector_evector_[i].size(); ++j)
        {
            BOOST_CHECK(configurable_out.std_nested_vector_evector_[i][j].isApprox(
                        configurable_in.std_nested_vector_evector_[i][j], g_tolerance));
        }
    }
}


#include "fixture_base_ros.h"
#include "fixture_000_basic_interface.h"
#include "fixture_001_constructor_interface.h"
#include "fixture_002_comparison.h"


// ===============================================================
// TESTS
// ===============================================================

#define ARILES_TESTS(NAMESPACE) \
    ARILES_FIXTURE_TEST_CASE(BasicInterfaceFixture, NAMESPACE, Configurable) \
    ARILES_FIXTURE_TEST_CASE(ConstructorInterfaceFixture, NAMESPACE, Configurable) \
    ARILES_FIXTURE_TEST_CASE(ComparisonSimpleFixture, NAMESPACE, Configurable)

ARILES_TESTS(ros)
