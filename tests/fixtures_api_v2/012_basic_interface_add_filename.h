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

                    typename t_Bridge::Writer writer(std::string("configurable") + ".cfg");
                    configurable.ariles(writer);
                }

                {
                    t_Configurable configurable;

                    typename t_Bridge::Reader reader(std::string("configurable") + ".cfg");
                    configurable.ariles(reader);
                }

                // --------------------------------

                // Implicit instantiation of reader and writer classes

                {
                    t_Configurable configurable;
                    configurable.randomize();
                    configurable.template ariles<typename t_Bridge::Writer>(std::string("configurable2") + ".cfg");
                }

                {
                    t_Configurable configurable;
                    configurable.template ariles<typename t_Bridge::Reader>(std::string("configurable2") + ".cfg");
                }


                // --------------------------------
                // strictness control
                // --------------------------------

                // Exlicit instantiation of reader and writer classes
                {
                    t_Configurable configurable;
                    configurable.randomize();

                    typename t_Bridge::Writer writer(std::string("configurable3") + ".cfg");
                    configurable.ariles(writer);
                }

                {
                    t_Configurable configurable;

                    typename t_Bridge::Reader reader(std::string("configurable3") + ".cfg");
                    configurable.ariles(reader,
                            ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                }

                // --------------------------------

                // Implicit instantiation of reader and writer classes

                {
                    t_Configurable configurable;
                    configurable.randomize();
                    configurable.template ariles<typename t_Bridge::Writer>(std::string("configurable4") + ".cfg");
                }

                {
                    t_Configurable configurable;
                    configurable.template ariles<typename t_Bridge::Reader>(std::string("configurable4") + ".cfg",
                            ariles::ConfigurableFlags::DEFAULT | ariles::ConfigurableFlags::ALLOW_MISSING_ENTRIES);
                }
            }
    };
}
