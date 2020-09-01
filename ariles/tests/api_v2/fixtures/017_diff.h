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
    class DiffFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            // Implicit instantiation of the writer class and compare with reference
            {
                t_Configurable configurable;
                ariles2::apply<ariles2::Defaults>(configurable);

                const std::string filename = std::string(ARILES_TESTS_TEST_NAME) + ".cfg";
                ariles2::apply<typename t_Visitor::Visitor>(getWriterInitializer(filename), configurable);

                const std::string ref_filename = std::string(ARILES_TESTS_TEST_NAME) + ".ref";
                const std::string diff_cmd = std::string("diff ") + filename + " " + ref_filename;
                BOOST_CHECK_EQUAL(0, std::system(diff_cmd.c_str()));
            }
        }
    };
}  // namespace ariles_tests
