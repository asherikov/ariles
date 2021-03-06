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
    template <class t_FixtureBase>
    class ComparisonViaBaseFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Base, class t_Configurable, class t_Visitor>
        void test()
        {
            t_Configurable configurable_out;
            configurable_out.randomize();
            t_Base *configurable_out_base = &configurable_out;
            BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Writer>(
                                         getWriterInitializer("configurable_match_member_definitions.cfg"),
                                         *configurable_out_base,
                                         "Configurable"););

            // -------

            t_Configurable configurable_in;
            t_Base *configurable_in_base = &configurable_in;
            BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Reader>(
                                         getReaderInitializer("configurable_match_member_definitions.cfg"),
                                         *configurable_in_base,
                                         "Configurable"););

            // -------

            compare(configurable_out, configurable_in);

            ariles2::Compare visitor;
            ariles2::Compare::Parameters param;
            param.double_tolerance_ = g_tolerance;
            param.compare_number_of_entries_ = true;
            BOOST_CHECK(ariles2::apply(visitor, configurable_out, configurable_in, param));
        }
    };
}  // namespace ariles_tests
