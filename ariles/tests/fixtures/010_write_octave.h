/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


template<class t_FixtureBase>
class OctaveFixture : public t_FixtureBase
{
    public:
        using t_FixtureBase::getWriterInitializer;


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

                std::string octave_cmd =
                    std::string("octave --no-gui --no-history --silent --eval 'source ")
                    + getWriterInitializer("configurable.cfg")
                    + "'";
                BOOST_CHECK_EQUAL(0, std::system(octave_cmd.c_str()));
            }

            // --------------------------------

            // Implicit instantiation of the writer class
            {
                t_Configurable configurable;
                configurable.randomize();
                configurable.template writeConfig<t_Bridge>(getWriterInitializer("configurable2.cfg"));

                std::string octave_cmd =
                    std::string("octave --no-gui --no-history --silent --eval 'source ")
                    + getWriterInitializer("configurable2.cfg")
                    + "'";
                BOOST_CHECK_EQUAL(0, std::system(octave_cmd.c_str()));
            }
        }
};
