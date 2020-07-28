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
    class ComparisonSimpleFixture : public t_FixtureBase
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
            BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Reader>(
                                         getReaderInitializer("configurable_match_simple.cfg"), configurable_in););

            // -------

            compare(configurable_out, configurable_in);

            ariles2::Compare visitor;
            ariles2::Compare::Parameters param;
            param.double_tolerance_ = g_tolerance;
            param.compare_number_of_entries_ = true;
            BOOST_CHECK(visitor.compare(configurable_out, configurable_in, param));
        }
    };


    template <class t_FixtureBase>
    class ComparisonMultiFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            t_Configurable configurable_out1;
            t_Configurable configurable_out2;

            configurable_out1.randomize();
            configurable_out2.randomize();

            BOOST_CHECK_NO_THROW(
                    typename t_Visitor::Writer writer(getWriterInitializer("configurable_match_multi.cfg"));
                    ariles2::apply(writer, configurable_out1, "node1");
                    ariles2::apply(writer, configurable_out2, "node2"););

            // -------

            t_Configurable configurable_in1;
            t_Configurable configurable_in2;

            BOOST_CHECK_NO_THROW(
                    typename t_Visitor::Reader reader(getReaderInitializer("configurable_match_multi.cfg"));
                    ariles2::apply(reader, configurable_in1, "node1");
                    ariles2::apply(reader, configurable_in2, "node2"););

            // -------

            compare(configurable_out1, configurable_in1);
            compare(configurable_out2, configurable_in2);

            ariles2::Compare visitor;
            ariles2::Compare::Parameters param;
            param.double_tolerance_ = g_tolerance;
            param.compare_number_of_entries_ = true;
            BOOST_CHECK(visitor.compare(configurable_out1, configurable_in1, param));
            BOOST_CHECK(visitor.compare(configurable_out2, configurable_in2, param));
        }
    };
}  // namespace ariles_tests
