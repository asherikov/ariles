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
        template <class t_Configurable>
        void compare(const t_Configurable &out, const t_Configurable &in)
        {
            ariles2::Compare visitor;
            ariles2::Compare::Parameters param;
            param.double_tolerance_ = g_tolerance;
            param.compare_number_of_entries_ = true;
            BOOST_CHECK(ariles2::apply(visitor, out, in, param));
        }


        template <class t_Configurable, class t_Visitor>
        void test()
        {
            typename t_Visitor::Writer::Parameters writer_parameters;
            writer_parameters.write_.allow_missing_entries_ = true;

            typename t_Visitor::Reader::Parameters reader_parameters;
            reader_parameters.read_.allow_missing_entries_ = true;

            {
                t_Configurable configurable_out;
                BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Writer>(
                                             getWriterInitializer("configurable_match_simple.cfg"),
                                             configurable_out,
                                             writer_parameters););

                // -------

                t_Configurable configurable_in;
                BOOST_CHECK_NO_THROW(ariles2::apply<typename t_Visitor::Reader>(
                                             getReaderInitializer("configurable_match_simple.cfg"),
                                             configurable_in,
                                             reader_parameters););

                // -------

                compare(configurable_out, configurable_in);
            }


            {
                t_Configurable configurable_out;
                configurable_out.setNull();

                ariles2::apply<typename t_Visitor::Writer>(
                        getWriterInitializer("configurable_match_simple.cfg"), configurable_out, writer_parameters);

                // -------

                t_Configurable configurable_in;
                configurable_in.setNull();

                ariles2::apply<typename t_Visitor::Reader>(
                        getReaderInitializer("configurable_match_simple.cfg"), configurable_in, reader_parameters);

                // -------

                compare(configurable_out, configurable_in);
            }
        }
    };
}  // namespace ariles_tests
