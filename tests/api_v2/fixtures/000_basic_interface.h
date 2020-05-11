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

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                ariles2::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                ariles2::apply(reader, configurable);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                ariles2::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
            }

            {
                t_Configurable configurable;
                ariles2::apply<typename t_Visitor::Reader>(getReaderInitializer("configurable2.cfg"), configurable);
            }


            // --------------------------------
            // strictness control
            // --------------------------------
            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
                ariles2::apply(writer, configurable);
            }

            {
                t_Configurable configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
                ariles2::apply(
                        reader,
                        configurable,
                        ariles2::ConfigurableFlags::DEFAULT | ariles2::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                ariles2::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
            }

            {
                t_Configurable configurable;
                ariles2::apply<typename t_Visitor::Reader>(
                        getReaderInitializer("configurable2.cfg"),
                        configurable,
                        ariles2::ConfigurableFlags::DEFAULT | ariles2::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
            }
        }
    };
}  // namespace ariles_tests
