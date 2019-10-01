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
            template<class t_ConfigurableBase, class t_Configurable, class t_Bridge>
                void test()
            {
                t_Configurable configurable_out;
                configurable_out.randomize();
                t_ConfigurableBase *configurable_out_base = &configurable_out;
                BOOST_CHECK_NO_THROW(
                    configurable_out_base->template writeConfig<t_Bridge>(getWriterInitializer("configurable_match_member_definitions.cfg"), "Configurable");
                );

                // -------

                t_Configurable configurable_in;
                t_ConfigurableBase *configurable_in_base = &configurable_in;
                BOOST_CHECK_NO_THROW(
                    configurable_in_base->template readConfig<t_Bridge>(getReaderInitializer("configurable_match_member_definitions.cfg"), "Configurable");
                );

                // -------

                compare(configurable_out, configurable_in);

                ariles::ComparisonParameters param;
                param.double_tolerance_ = g_tolerance;
                param.compare_number_of_entries_ = true;
                param.throw_on_error_ = true;
                BOOST_CHECK(configurable_out.arilesCompare(configurable_in, param));
            }
    };
}
