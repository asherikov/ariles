/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


class ComparisonSimpleFixture
{
    protected:
        template<class t_Configurable, class t_Reader, class t_Writer>
            void test()
        {
            t_Configurable  configurable_out;
            BOOST_CHECK_NO_THROW(
                configurable_out.template writeConfig<t_Writer>("configurable_match_simple.cfg");
            );

            // -------

            t_Configurable  configurable_in;
            BOOST_CHECK_NO_THROW(
                configurable_in.template readConfig<t_Reader>("configurable_match_simple.cfg");
            );

            // -------

            compare(configurable_out, configurable_in);
        }
};


class ComparisonMultiFixture
{
    protected:
        template<class t_Configurable, class t_Reader, class t_Writer>
            void test()
        {
            t_Configurable configurable_out1;
            t_Configurable configurable_out2;

            BOOST_CHECK_NO_THROW(
                t_Writer writer("configurable_match_multi.cfg");
                configurable_out1.template writeConfig(writer, "node1");
                configurable_out2.template writeConfig(writer, "node2");
            );

            // -------

            t_Configurable configurable_in1;
            t_Configurable configurable_in2;

            BOOST_CHECK_NO_THROW(
                t_Reader reader("configurable_match_multi.cfg");
                configurable_in1.template readConfig(reader, "node1");
                configurable_in2.template readConfig(reader, "node2");
            );

            // -------

            compare(configurable_out1, configurable_in1);
            compare(configurable_out2, configurable_in2);
        }
};
