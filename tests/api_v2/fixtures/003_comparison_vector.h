/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    template <class t_Configurable>
    class ConfigurableVector : public ARILES_TEST_DEFAULT_BASE
    {
#define ARILES2_ENTRIES(v) ARILES2_ENTRY_(v, vector)
#include ARILES2_INITIALIZE

    public:
        std::vector<t_Configurable> vector_;


    public:
        ConfigurableVector()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
        }


        void randomize()
        {
            vector_.resize(4);

            for (std::size_t i = 0; i < vector_.size(); ++i)
            {
                vector_[i].randomize();
            }
        }
    };


    template <class t_FixtureBase>
    class ComparisonVectorFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            ConfigurableVector<t_Configurable> configurable_vector_out;
            configurable_vector_out.randomize();
            BOOST_CHECK_NO_THROW(
                    ariles2::apply<typename t_Visitor::Writer>(
                            getWriterInitializer("configurable_match_vector.cfg"), configurable_vector_out););

            // -------

            ConfigurableVector<t_Configurable> configurable_vector_in;
            BOOST_CHECK_NO_THROW(
                    ariles2::apply<typename t_Visitor::Reader>(
                            getReaderInitializer("configurable_match_vector.cfg"), configurable_vector_in););

            // -------

            BOOST_REQUIRE_EQUAL(configurable_vector_out.vector_.size(), configurable_vector_in.vector_.size());
            for (std::size_t i = 0; i < configurable_vector_out.vector_.size(); ++i)
            {
                compare(configurable_vector_out.vector_[i], configurable_vector_in.vector_[i]);
            }

            ariles2::Compare visitor;
            ariles2::Compare::Parameters param;
            param.double_tolerance_ = g_tolerance;
            param.compare_number_of_entries_ = true;
            param.throw_on_error_ = true;
            BOOST_CHECK(visitor.compare(configurable_vector_out, configurable_vector_in, param));
        }
    };
}  // namespace ariles_tests
