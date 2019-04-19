/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


template<class t_FixtureBase>
class ComparisonEquivalenceFixture : public t_FixtureBase
{
    public:
        using t_FixtureBase::getWriterInitializer;
        using t_FixtureBase::getReaderInitializer;


    protected:
        template<class t_Configurable1, class t_Configurable2, class t_Bridge>
            void test()
        {
            {
                t_Configurable1 configurable_out;
                configurable_out.randomize();
                BOOST_CHECK_NO_THROW(
                    configurable_out.template writeConfig<t_Bridge>(getWriterInitializer("configurable_match_member_definitions.cfg"), "Configurable");
                );

                // -------

                t_Configurable2 configurable_in;
                BOOST_CHECK_NO_THROW(
                    configurable_in.template readConfig<t_Bridge>(getReaderInitializer("configurable_match_member_definitions.cfg"), "Configurable");
                );

                // -------

                compare(configurable_out, configurable_in);
            }

            // -------
            // -------
            // -------

            {
                t_Configurable2 configurable_out;
                configurable_out.randomize();
                BOOST_CHECK_NO_THROW(
                    configurable_out.template writeConfig<t_Bridge>(getWriterInitializer("configurable_match_member_definitions.cfg"), "Configurable");
                );

                // -------

                t_Configurable1 configurable_in;
                BOOST_CHECK_NO_THROW(
                    configurable_in.template readConfig<t_Bridge>(getReaderInitializer("configurable_match_member_definitions.cfg"), "Configurable");
                );

                // -------

                compare(configurable_out, configurable_in);
            }
        }
};
