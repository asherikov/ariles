/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    template <class t_FixtureBase>
    class PythonFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            // Explicit instantiation of writer class
            {
                t_Configurable configurable;
                configurable.randomize();

                const std::string filename =
                        std::string("python_") + configurable.arilesDefaultID() + "_configurable.py";
                typename t_Visitor::Writer writer(getWriterInitializer(filename));
                ariles2::apply(writer, configurable);

                // Try to execute the generated Python script to validate it can be loaded with numpy
                std::string python_cmd = std::string("python3 -c \"import sys; import os; sys.path.insert(0, os.path.dirname('")
                                         + getWriterInitializer(filename) + "')); exec(open('" + getWriterInitializer(filename) + "').read())\"";
                BOOST_CHECK_EQUAL(0, std::system(python_cmd.c_str()));
            }

            // --------------------------------

            // Implicit instantiation of the writer class
            {
                t_Configurable configurable;
                configurable.randomize();
                const std::string filename =
                        std::string("python_") + configurable.arilesDefaultID() + "_configurable2.py";
                ariles2::apply<typename t_Visitor::Writer>(getWriterInitializer(filename), configurable);

                // Try to execute the generated Python script to validate it can be loaded with numpy
                std::string python_cmd = std::string("python3 -c \"import sys; import os; sys.path.insert(0, os.path.dirname('")
                                         + getWriterInitializer(filename) + "')); exec(open('" + getWriterInitializer(filename) + "').read())\"";
                BOOST_CHECK_EQUAL(0, std::system(python_cmd.c_str()));
            }
        }
    };
}  // namespace ariles_tests