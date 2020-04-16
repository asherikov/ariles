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
    template<class t_FixtureBase>
    class ComparisonViaBaseFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Base, class t_Configurable, class t_Visitor>
                void test()
            {
                t_Configurable configurable_out;
                configurable_out.randomize();
                t_Base *configurable_out_base = &configurable_out;
                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable_match_member_definitions.cfg"), *configurable_out_base, "Configurable");
                );

                // -------

                t_Configurable configurable_in;
                t_Base *configurable_in_base = &configurable_in;
                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Visitor::Reader>(getReaderInitializer("configurable_match_member_definitions.cfg"), *configurable_in_base, "Configurable");
                );

                // -------

                compare(configurable_out, configurable_in);

                ariles::Compare visitor;
                ariles::Compare::Parameters param;
                param.double_tolerance_ = g_tolerance;
                param.compare_number_of_entries_ = true;
                param.throw_on_error_ = true;
                BOOST_CHECK(visitor.compare(configurable_out, configurable_in, param));
            }
    };
}
