/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


class ConstructorInterfaceFixture : public FixtureBase
{
    protected:
        template<class t_Configurable, class t_Reader, class t_Writer>
            void test()
        {
            BOOST_CHECK_NO_THROW(
                t_Configurable configurable;
                configurable.randomize();

                t_Writer writer(getInitializer("configurable.cfg"));
                configurable.writeConfig(writer);
            );

            BOOST_CHECK_NO_THROW(
                t_Reader reader(getInitializer("configurable.cfg"));
                t_Configurable configurable(reader);
            );
        }
};
