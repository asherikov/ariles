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
            // Explicit instantiation of reader and writer classes
            {
                t_Configurable configurable;
                configurable.randomize();

                const std::string filename =
                        std::string("graphviz_") + configurable.arilesDefaultID() + "_configurable.cfg";
                typename t_Visitor::Visitor writer(getWriterInitializer(filename));
                ariles2::apply(writer, configurable);

                std::string dot_cmd = std::string("dot -Tjpg -O ") + getWriterInitializer(filename);
                BOOST_CHECK_EQUAL(0, std::system(dot_cmd.c_str()));
            }

            // --------------------------------

            // Implicit instantiation of the writer class
            {
                t_Configurable configurable;
                configurable.randomize();

                const std::string filename =
                        std::string("graphviz_") + configurable.arilesDefaultID() + "_configurable2.cfg";
                ariles2::apply<typename t_Visitor::Visitor>(getWriterInitializer(filename), configurable);

                std::string dot_cmd = std::string("dot -Tjpg -O ") + getWriterInitializer(filename);
                BOOST_CHECK_EQUAL(0, std::system(dot_cmd.c_str()));
            }
        }
    };
}  // namespace ariles_tests
