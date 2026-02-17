/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    namespace
    {
        struct Complete : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, member_1, int)                                                                             \
    ARILES2_TYPED_ENTRY_(v, member_2, int)
#include ARILES2_INITIALIZE
        };

        struct Partial : public ariles2::DefaultBase
        {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, member_2, int)
#include ARILES2_INITIALIZE
        };
    }  // namespace


    template <class t_FixtureBase>
    class PartialEquivalenceFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_ConfigurableOut, class t_ConfigurableIn, class t_Visitor>
        void test()
        {
            {
                t_ConfigurableOut configurable_out;
                configurable_out.randomize();
                BOOST_CHECK_NO_THROW(
                        ariles2::apply<typename t_Visitor::Writer>(
                                getWriterInitializer("configurable_match_partial_definitions.cfg"), configurable_out););

                // -------

                t_ConfigurableIn configurable_in;
                ariles2::read::Parameters parameters;
                parameters.allow_missing_entries_ = true;
                ariles2::apply<typename t_Visitor::Reader>(
                        getReaderInitializer("configurable_match_partial_definitions.cfg"),
                        configurable_in,
                        parameters);

                // -------

                ariles2::Compare visitor;
                ariles2::Compare::Parameters param;
                param.double_tolerance_ = g_tolerance;
                compareAndCheckWithBacktrace(
                        visitor, configurable_out, configurable_in, param, "ComparisonMultiFixture 1 failed");
            }
        }
    };
}  // namespace ariles_tests
