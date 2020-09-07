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
    class SetDefaultsCheckFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                configurable.writeConfig(writer);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.set_defaults_check_flag_ = false;
                configurable.readConfig(reader);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.template writeConfig<t_Visitor>(getWriterInitializer("configurable2.cfg"));
            }

            {
                t_Configurable configurable;
                configurable.set_defaults_check_flag_ = false;
                configurable.template readConfig<t_Visitor>(getReaderInitializer("configurable2.cfg"));
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }


            // --------------------------------
            // strictness control
            // --------------------------------
            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                configurable.writeConfig(writer);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.set_defaults_check_flag_ = false;
                configurable.readConfig(
                        reader, ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.template writeConfig<t_Visitor>(getWriterInitializer("configurable2.cfg"));
            }

            {
                t_Configurable configurable;
                configurable.set_defaults_check_flag_ = false;
                configurable.template readConfig<t_Visitor>(
                        getReaderInitializer("configurable2.cfg"),
                        ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }
        }
    };
}  // namespace ariles_tests
