/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


template<class t_Configurable_out, class t_Configurable_in>
void    compare(const t_Configurable_out    &configurable_out,
                const t_Configurable_in     &configurable_in)
{
    BOOST_CHECK_EQUAL(configurable_out.integer_,          configurable_in.integer_);
    BOOST_CHECK_EQUAL(configurable_out.unsigned_integer_,          configurable_in.unsigned_integer_);
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
