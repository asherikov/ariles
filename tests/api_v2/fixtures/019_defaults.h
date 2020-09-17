/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
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
            // Explicit instantiation of reader and writer classes
            {
                t_Configurable configurable;

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                ariles2::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.defaults_check_flag_ = false;
                ariles2::apply(reader, configurable);
                BOOST_CHECK(configurable.defaults_check_flag_);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                ariles2::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
            }

            {
                t_Configurable configurable;
                configurable.defaults_check_flag_ = false;
                ariles2::apply<typename t_Visitor::Reader>(getReaderInitializer("configurable2.cfg"), configurable);
                BOOST_CHECK(configurable.defaults_check_flag_);
            }


            // --------------------------------
            // strictness control
            // --------------------------------
            // Explicit instantiation of reader and writer classes
            {
                t_Configurable configurable;

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                ariles2::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                configurable.defaults_check_flag_ = false;
                typename t_Visitor::Reader::Parameters parameters;
                parameters.read_.override_parameters_ = false;
                parameters.read_.allow_missing_entries_ = true;
                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                ariles2::apply(reader, configurable, parameters);
                BOOST_CHECK(configurable.defaults_check_flag_);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                ariles2::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
            }

            {
                t_Configurable configurable;
                configurable.defaults_check_flag_ = false;
                ariles2::read::Parameters parameters;
                parameters.override_parameters_ = false;
                parameters.allow_missing_entries_ = true;
                ariles2::apply<typename t_Visitor::Reader>(
                        getReaderInitializer("configurable2.cfg"), configurable, parameters);
                BOOST_CHECK(configurable.defaults_check_flag_);
            }
        }
    };
}  // namespace ariles_tests
