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
    template<class t_FixtureBase>
    class BasicInterfaceFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                // Exlicit instantiation of reader and writer classes
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    typename t_Bridge::Writer writer(getWriterInitializer("configurable.cfg"));
                    ariles::apply(writer, configurable);
                }

                {
                    t_Configurable configurable;

                    typename t_Bridge::Reader reader(getReaderInitializer("configurable.cfg"));
                    ariles::apply(reader, configurable);
                }

                // --------------------------------

                // Implicit instantiation of reader and writer classes

                {
                    t_Configurable configurable;
                    configurable.randomize();
                    ariles::apply<typename t_Bridge::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
                }

                {
                    t_Configurable configurable;
                    ariles::apply<typename t_Bridge::Reader>(getReaderInitializer("configurable2.cfg"), configurable);
                }


                // --------------------------------
                // strictness control
                // --------------------------------
                // Exlicit instantiation of reader and writer classes
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    typename t_Bridge::Writer writer(getWriterInitializer("configurable.cfg"));
                    ariles::apply(writer, configurable);
                }

                {
                    t_Configurable configurable;

                    typename t_Bridge::Reader reader(getReaderInitializer("configurable.cfg"));
                    ariles::apply(reader, configurable,
                            ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                }

                // --------------------------------

                // Implicit instantiation of reader and writer classes

                {
                    t_Configurable configurable;
                    configurable.randomize();
                    ariles::apply<typename t_Bridge::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
                }

                {
                    t_Configurable configurable;
                    ariles::apply<typename t_Bridge::Reader>(getReaderInitializer("configurable2.cfg"), configurable,
                            ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                }
            }
    };
}
