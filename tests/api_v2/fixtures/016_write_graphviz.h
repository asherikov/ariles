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
    class GraphvizFixture : public t_FixtureBase
    {
    public:
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

                std::string dot_cmd = std::string("dot -Tjpg -o configurable.jpg ")
                                         + getWriterInitializer("configurable.cfg");
                BOOST_CHECK_EQUAL(0, std::system(dot_cmd.c_str()));
            }

            // --------------------------------

            // Implicit instantiation of the writer class
            {
                t_Configurable configurable;
                configurable.randomize();
                ariles2::apply<typename t_Visitor::Writer>(getWriterInitializer("configurable2.cfg"), configurable);

                std::string dot_cmd = std::string("dot -Tjpg -o configurable2.jpg ")
                                         + getWriterInitializer("configurable.cfg");
                BOOST_CHECK_EQUAL(0, std::system(dot_cmd.c_str()));
            }
        }
    };
}  // namespace ariles_tests
