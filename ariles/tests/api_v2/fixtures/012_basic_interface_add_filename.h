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
            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                typename t_Visitor::Writer writer(std::string("configurable") + ".cfg");
                ariles::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(std::string("configurable") + ".cfg");
                ariles::apply(reader, configurable);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                ariles::apply<typename t_Visitor::Writer>(std::string("configurable2") + ".cfg", configurable);
            }

            {
                t_Configurable configurable;
                ariles::apply<typename t_Visitor::Reader>(std::string("configurable2") + ".cfg", configurable);
            }


            // --------------------------------
            // strictness control
            // --------------------------------

            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                typename t_Visitor::Writer writer(std::string("configurable3") + ".cfg");
                ariles::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(std::string("configurable3") + ".cfg");
                ariles::apply(
                        reader,
                        configurable,
                        ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                ariles::apply<typename t_Visitor::Writer>(std::string("configurable4") + ".cfg", configurable);
            }

            {
                t_Configurable configurable;
                ariles::apply<typename t_Visitor::Reader>(
                        std::string("configurable4") + ".cfg",
                        configurable,
                        ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
            }
        }
    };
}  // namespace ariles_tests
