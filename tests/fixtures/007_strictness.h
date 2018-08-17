/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


template<class t_FixtureBase>
class StrictnessFixture : public t_FixtureBase
{
    public:
        using t_FixtureBase::getWriterInitializer;
        using t_FixtureBase::getReaderInitializer;


    protected:
        template<class t_Configurable1, class t_Configurable2, class t_Bridge>
            void test()
        {
            // Exlicit instantiation of reader and writer classes
            BOOST_CHECK_NO_THROW(
                t_Configurable1 configurable;

                typename t_Bridge::Writer writer(getWriterInitializer("configurable.cfg"));
                configurable.writeConfig(writer);
            );

            BOOST_CHECK_THROW(
                t_Configurable2 configurable;

                typename t_Bridge::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.readConfig(reader);
                ,
                std::runtime_error
            );

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            BOOST_CHECK_NO_THROW(
                t_Configurable1 configurable;
                configurable.template writeConfig<t_Bridge>(getWriterInitializer("configurable2.cfg"));
            );

            BOOST_CHECK_THROW(
                t_Configurable2 configurable;
                configurable.template readConfig<t_Bridge>(getReaderInitializer("configurable2.cfg"));
                ,
                std::runtime_error
            );
        }
};
