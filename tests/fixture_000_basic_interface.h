/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


class BasicInterfaceFixture
{
    protected:
        template<class t_Configurable, class t_Reader, class t_Writer>
            void test()
        {
            // Exlicit instantiation of reader and writer classes
            BOOST_CHECK_NO_THROW(
                t_Configurable configurable;

                t_Writer writer("configurable.cfg");
                configurable.writeConfig(writer);
            );

            BOOST_CHECK_NO_THROW(
                t_Configurable configurable;

                t_Reader reader("configurable.cfg");
                configurable.readConfig(reader);
            );

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            BOOST_CHECK_NO_THROW(
                t_Configurable configurable;
                configurable.template writeConfig<t_Writer>("configurable2.cfg");
            );

            BOOST_CHECK_NO_THROW(
                t_Configurable configurable;
                configurable.template readConfig<t_Reader>("configurable2.cfg");
            );
        }
};
