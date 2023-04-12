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
    class BasicInterfaceFixture : public t_FixtureBase
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
                configurable.randomize();

                typename t_Visitor::Writer writer(std::string("configurable") + ".cfg");
                ariles2::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(std::string("configurable") + ".cfg");
                ariles2::apply(reader, configurable);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                ariles2::apply<typename t_Visitor::Writer>(std::string("configurable2") + ".cfg", configurable);
            }

            {
                t_Configurable configurable;
                ariles2::apply<typename t_Visitor::Reader>(std::string("configurable2") + ".cfg", configurable);
            }


            // --------------------------------
            // strictness control
            // --------------------------------

            // Explicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                typename t_Visitor::Writer::Parameters parameters;
                parameters.write_.allow_missing_entries_ = true;

                typename t_Visitor::Writer writer(std::string("configurable3") + ".cfg");
                ariles2::apply(writer, configurable, parameters);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader::Parameters parameters;
                parameters.read_.override_parameters_ = true;
                parameters.read_.allow_missing_entries_ = true;
                typename t_Visitor::Reader reader(std::string("configurable3") + ".cfg");
                ariles2::apply(reader, configurable, parameters);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();

                typename t_Visitor::Writer::Parameters parameters;
                parameters.write_.allow_missing_entries_ = true;

                ariles2::apply<typename t_Visitor::Writer>(
                        std::string("configurable4") + ".cfg", configurable, parameters);
            }

            {
                t_Configurable configurable;
                typename t_Visitor::Reader::Parameters parameters;
                parameters.read_.override_parameters_ = true;
                parameters.read_.allow_missing_entries_ = true;
                ariles2::apply<typename t_Visitor::Reader>(
                        std::string("configurable4") + ".cfg", configurable, parameters);
            }
        }
    };
}  // namespace ariles_tests
