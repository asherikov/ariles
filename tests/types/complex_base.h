/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "enum.h"

template <class t_ConfigurableComplex>
class ConfigurableComplexBase
{
    protected:
        ConfigurableComplexBase() {}
        ~ConfigurableComplexBase() {}


    public:
        void setDefaults()
        {
            t_ConfigurableComplex &impl = static_cast<t_ConfigurableComplex &>(*this);


            impl.integer_ = 10;
            impl.unsigned_integer_ = 100;
            impl.real_ = 1.33;
            impl.string_ = "blahblah";

            impl.std_vector_.resize(5);
            for(std::size_t i = 0; i < impl.std_vector_.size(); ++i)
            {
                impl.std_vector_[i] = i * 5.22 + 2.3;
            }

            impl.std_nested_vector_.resize(3);
            for(std::size_t i = 0; i < impl.std_nested_vector_.size(); ++i)
            {
                impl.std_nested_vector_[i].resize(7-i);

                for(std::size_t j = 0; j < impl.std_nested_vector_[i].size(); ++j)
                {
                    impl.std_nested_vector_[i][j] = 5.2 + i*0.1 + j*0.3;
                }
            }

            impl.boolean_false_ = false;
            impl.boolean_true_ = true;

            impl.enum_ = ANOTHER_VALUE;
            impl.better_enum_ = BetterEnum::DEFINED_1;

            impl.std_pair_.first = "test";
            impl.std_pair_.second = 13;

            std::vector<std::string> std_map_test;
            std_map_test.push_back("one");
            impl.std_map_["one"] = std_map_test;
            std_map_test.push_back("two");
            impl.std_map_["two"] = std_map_test;
            std_map_test.push_back("three");
            impl.std_map_["three"] = std_map_test;


#ifdef ARILES_ADAPTER_EIGEN
            impl.vector_.setConstant(3);
            impl.matrix_ << 1, 2, 3, 4, 5, 6, 7, 8, 9;
            impl.matrix_x_.resize(2, 3);
            impl.matrix_x_ << 8, 7, 6, 3, 2, 1;

            impl.std_vector_evector_.resize(4);
            for(std::size_t i = 0; i < impl.std_vector_evector_.size(); ++i)
            {
                impl.std_vector_evector_[i].setConstant(i*9 + 0.43);
            }

            impl.std_nested_vector_evector_.resize(2);
            for(std::size_t i = 0; i < impl.std_nested_vector_evector_.size(); ++i)
            {
                impl.std_nested_vector_evector_[i].resize(1+i);

                for(std::size_t j = 0; j < impl.std_nested_vector_evector_[i].size(); ++j)
                {
                    impl.std_nested_vector_evector_[i][j].setConstant(5.2 + i*0.1 + j*0.3);
                }
            }

            impl.isometry_.setIdentity();

            impl.quaternion_.setIdentity();
#endif
        }


#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
        void randomize()
        {
            t_ConfigurableComplex &impl = static_cast<t_ConfigurableComplex &>(*this);

            impl.integer_ = GET_RANDOM_INT;
            impl.unsigned_integer_ = GET_RANDOM_UINT;
            impl.real_ = GET_RANDOM_REAL;
            impl.string_ = "blahblah";

            impl.std_vector_.resize(5);
            for(std::size_t i = 0; i < impl.std_vector_.size(); ++i)
            {
                impl.std_vector_[i] = GET_RANDOM_REAL;
            }

            impl.std_nested_vector_.resize(3);
            for(std::size_t i = 0; i < impl.std_nested_vector_.size(); ++i)
            {
                impl.std_nested_vector_[i].resize(7-i);

                for(std::size_t j = 0; j < impl.std_nested_vector_[i].size(); ++j)
                {
                    impl.std_nested_vector_[i][j] = GET_RANDOM_REAL;
                }
            }

            impl.boolean_false_ = false;
            impl.boolean_true_ = true;

            impl.enum_ = ANOTHER_VALUE;
            impl.better_enum_ = BetterEnum::DEFINED_2;

            impl.std_pair_.first = "testtt";
            impl.std_pair_.second = GET_RANDOM_REAL;

            impl.std_map_.clear();
            std::vector<std::string> std_map_test;
            std_map_test.push_back("1one");
            impl.std_map_["one1"] = std_map_test;
            std_map_test.push_back("2two");
            impl.std_map_["two2"] = std_map_test;
            std_map_test.push_back("3three");
            impl.std_map_["three3"] = std_map_test;
            std_map_test.push_back("4four");
            impl.std_map_["four4"] = std_map_test;


#   ifdef ARILES_ADAPTER_EIGEN
            impl.vector_.setRandom();
            impl.matrix_.setRandom();
            impl.matrix_x_.resize(2, 3);
            impl.matrix_x_.setRandom();

            impl.std_vector_evector_.resize(4);
            for(std::size_t i = 0; i < impl.std_vector_evector_.size(); ++i)
            {
                impl.std_vector_evector_[i].setRandom();
            }

            impl.std_nested_vector_evector_.resize(2);
            for(std::size_t i = 0; i < impl.std_nested_vector_evector_.size(); ++i)
            {
                impl.std_nested_vector_evector_[i].resize(1+i);

                for(std::size_t j = 0; j < impl.std_nested_vector_evector_[i].size(); ++j)
                {
                    impl.std_nested_vector_evector_[i][j].setRandom();
                }
            }

            impl.isometry_.matrix() = Eigen::MatrixXd::Random(4,4);

            impl.quaternion_.x() = GET_RANDOM_REAL;
            impl.quaternion_.y() = GET_RANDOM_REAL;
            impl.quaternion_.z() = GET_RANDOM_REAL;
            impl.quaternion_.w() = GET_RANDOM_REAL;
#   endif

            impl.finalize();
        }
#endif
};



#ifndef ARILES_TESTS_BOOST_UTF_DISABLED
template<class t_Configurable_out, class t_Configurable_in>
void    compare(const t_Configurable_out    &configurable_out,
                const t_Configurable_in     &configurable_in)
{
    BOOST_CHECK_EQUAL(configurable_out.integer_,          configurable_in.integer_);
    BOOST_CHECK_EQUAL(configurable_out.unsigned_integer_, configurable_in.unsigned_integer_);
    BOOST_CHECK_CLOSE(configurable_out.real_,             configurable_in.real_, g_tolerance);
    BOOST_CHECK_EQUAL(configurable_out.string_,           configurable_in.string_);
    BOOST_CHECK_EQUAL(configurable_out.boolean_false_,    configurable_in.boolean_false_);
    BOOST_CHECK_EQUAL(configurable_out.boolean_false_,    false);
    BOOST_CHECK_EQUAL(configurable_out.boolean_true_,     configurable_in.boolean_true_);
    BOOST_CHECK_EQUAL(configurable_out.boolean_true_,     true);
    BOOST_CHECK_EQUAL(configurable_out.enum_,             configurable_in.enum_);
    BOOST_CHECK_EQUAL(configurable_out.better_enum_,      configurable_in.better_enum_);

    BOOST_CHECK_EQUAL(configurable_out.std_vector_.size(),                configurable_in.std_vector_.size());
    BOOST_CHECK_EQUAL(configurable_out.std_nested_vector_.size(),         configurable_in.std_nested_vector_.size());

    for (std::size_t i = 0; i < configurable_out.std_vector_.size(); ++i)
    {
        BOOST_CHECK_CLOSE(configurable_out.std_vector_[i],
                    configurable_in.std_vector_[i],
                    g_tolerance);
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


    BOOST_CHECK_EQUAL(configurable_out.std_pair_.first,     configurable_in.std_pair_.first);
    BOOST_CHECK_CLOSE(configurable_out.std_pair_.second,    configurable_in.std_pair_.second, g_tolerance);


    BOOST_CHECK_EQUAL(configurable_out.std_map_.size(),     configurable_in.std_map_.size());

    for (   std::map<std::string, std::vector<std::string> >::const_iterator it = configurable_in.std_map_.begin();
            it != configurable_in.std_map_.end();
            ++it)
    {
        std::map<std::string, std::vector<std::string> >::const_iterator search = configurable_out.std_map_.find(it->first);
        BOOST_REQUIRE(search != configurable_out.std_map_.end());

        BOOST_CHECK_EQUAL(it->second.size(), search->second.size());
        for (std::size_t i = 0; i < it->second.size(); ++i)
        {
            BOOST_CHECK_EQUAL(it->second[i], search->second[i]);
        }
    }


#   ifdef ARILES_ADAPTER_EIGEN
    BOOST_CHECK(configurable_out.vector_.isApprox(configurable_in.vector_, g_tolerance));
    BOOST_CHECK(configurable_out.matrix_.isApprox(configurable_in.matrix_, g_tolerance));
    BOOST_CHECK(configurable_out.matrix_x_.isApprox(configurable_in.matrix_x_, g_tolerance));
    BOOST_CHECK_EQUAL(configurable_out.std_vector_evector_.size(),        configurable_in.std_vector_evector_.size());
    BOOST_CHECK_EQUAL(configurable_out.std_nested_vector_evector_.size(), configurable_in.std_nested_vector_evector_.size());

    for (std::size_t i = 0; i < configurable_out.std_vector_evector_.size(); ++i)
    {
        BOOST_CHECK(configurable_out.std_vector_evector_[i].isApprox(
                    configurable_in.std_vector_evector_[i], g_tolerance));
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

    BOOST_CHECK(configurable_out.isometry_.isApprox(configurable_in.isometry_, g_tolerance));
    BOOST_CHECK(configurable_out.quaternion_.isApprox(configurable_in.quaternion_, g_tolerance));
#   endif
}
#endif
