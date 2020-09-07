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
    class DefaultsCheckFixture : public t_FixtureBase
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
                ariles::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.set_defaults_check_flag_ = false;
                ariles::apply(reader, configurable);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                ariles::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
            }

            {
                t_Configurable configurable;
                configurable.set_defaults_check_flag_ = false;
                ariles::apply<typename t_Visitor::Reader>(getReaderInitializer("configurable2.cfg"), configurable);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }


            // --------------------------------
            // strictness control
            // --------------------------------
            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                ariles::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.set_defaults_check_flag_ = false;
                ariles::apply(
                        reader,
                        configurable,
                        ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                ariles::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
            }

            {
                t_Configurable configurable;
                configurable.set_defaults_check_flag_ = false;
                ariles::apply<typename t_Visitor::Reader>(
                        getReaderInitializer("configurable2.cfg"),
                        configurable,
                        ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                BOOST_CHECK(configurable.set_defaults_check_flag_);
            }
        }
    };
}  // namespace ariles_tests
