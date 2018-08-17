/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


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
                configurable.writeConfig(writer);
            }

            {
                t_Configurable configurable;

                typename t_Bridge::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.readConfig(reader);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                configurable.template writeConfig<t_Bridge>(getWriterInitializer("configurable2.cfg"));
            }

            {
                t_Configurable configurable;
                configurable.template readConfig<t_Bridge>(getReaderInitializer("configurable2.cfg"));
            }


            // --------------------------------
            // strictness control
            // --------------------------------
            bool strict = true;

            // Exlicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                typename t_Bridge::Writer writer(getWriterInitializer("configurable.cfg"));
                configurable.writeConfig(writer, strict);
            }

            {
                t_Configurable configurable;

                typename t_Bridge::Reader reader(getReaderInitializer("configurable.cfg"));
                configurable.readConfig(reader, strict);
            }

            // --------------------------------

            // Implicit instantiation of reader and writer classes

            {
                t_Configurable configurable;
                configurable.randomize();
                configurable.template writeConfig<t_Bridge>(getWriterInitializer("configurable2.cfg"), strict);
            }

            {
                t_Configurable configurable;
                configurable.template readConfig<t_Bridge>(getReaderInitializer("configurable2.cfg"), strict);
            }
        }
};
