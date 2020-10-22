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
    class ComparisonSubtreeFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            t_Configurable configurable_out;
            configurable_out.randomize();
            BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Writer>(
                                         getWriterInitializer("configurable_match_simple.cfg"), configurable_out););

            // -------

            t_Configurable configurable_in;
            std::vector<std::string> subtree;
            subtree.push_back("");
            subtree.push_back("another_member1");
            subtree.push_back("member");
            BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Reader>(
                                         getReaderInitializer("configurable_match_simple.cfg"), configurable_in.another_member1_.member_, subtree););

            // -------

            ariles2::Compare visitor;
            ariles2::Compare::Parameters param;
            param.double_tolerance_ = g_tolerance;
            param.compare_number_of_entries_ = true;
            BOOST_CHECK(ariles2::apply(visitor, configurable_out.another_member1_.member_, configurable_in.another_member1_.member_, param));
        }
    };
}  // namespace ariles_tests
