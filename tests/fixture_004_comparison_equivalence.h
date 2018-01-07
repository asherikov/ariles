/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


class ComparisonEquivalenceFixture
{
    protected:
        template<class t_Configurable1, class t_Configurable2, class t_Reader, class t_Writer>
            void test()
        {
            {
                t_Configurable1 configurable_out;
                BOOST_CHECK_NO_THROW(
                    configurable_out.template writeConfig<t_Writer>("configurable_match_member_definitions.cfg", "Configurable");
                );

                // -------

                t_Configurable2 configurable_in;
                BOOST_CHECK_NO_THROW(
                    configurable_in.template readConfig<t_Reader>("configurable_match_member_definitions.cfg", "Configurable");
                );

                // -------

                compare(configurable_out, configurable_in);
            }

            // -------
            // -------
            // -------

            {
                t_Configurable2 configurable_out;
                BOOST_CHECK_NO_THROW(
                    configurable_out.template writeConfig<t_Writer>("configurable_match_member_definitions.cfg", "Configurable");
                );

                // -------

                t_Configurable1 configurable_in;
                BOOST_CHECK_NO_THROW(
                    configurable_in.template readConfig<t_Reader>("configurable_match_member_definitions.cfg", "Configurable");
                );

                // -------

                compare(configurable_out, configurable_in);
            }
        }
};
