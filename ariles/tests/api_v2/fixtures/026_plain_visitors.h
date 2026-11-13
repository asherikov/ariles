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
    class PlainVisitorFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;


    protected:
        template <class t_Visitor>
        // cppcheck-suppress functionStatic
        void testPlain()
        {
            ConfigurablePlain configurable;
            configurable.integer_ = 42;
            configurable.real_ = 1.5;

            {
                typename t_Visitor::PlainWriter writer(getWriterInitializer("configurable_plain.cfg"));
                ariles2::apply(writer, configurable);
                BOOST_CHECK(not configurable.prewrite_visited_);
            }

            {
                ConfigurablePlain read_configurable;

                typename t_Visitor::PlainReader reader(getReaderInitializer("configurable_plain.cfg"));
                ariles2::apply(reader, read_configurable);

                BOOST_CHECK_EQUAL(read_configurable.integer_, 42);
                BOOST_CHECK_CLOSE(read_configurable.real_, 1.5, g_tolerance);
                BOOST_CHECK(not read_configurable.defaults_visited_);
                BOOST_CHECK(not read_configurable.finalize_visited_);
                BOOST_CHECK(not read_configurable.prewrite_visited_);
            }


            // Implicit instantiation of plain reader and writer classes

            {
                configurable.integer_ = 42;
                configurable.real_ = 1.5;

                ariles2::apply<typename t_Visitor::PlainWriter>(
                        getWriterInitializer("configurable_plain2.cfg"), configurable);
                BOOST_CHECK(not configurable.prewrite_visited_);
            }

            {
                ConfigurablePlain read_configurable;

                ariles2::apply<typename t_Visitor::PlainReader>(
                        getReaderInitializer("configurable_plain2.cfg"), read_configurable);

                BOOST_CHECK_EQUAL(read_configurable.integer_, 42);
                BOOST_CHECK_CLOSE(read_configurable.real_, 1.5, g_tolerance);
                BOOST_CHECK(not read_configurable.defaults_visited_);
                BOOST_CHECK(not read_configurable.finalize_visited_);
                BOOST_CHECK(not read_configurable.prewrite_visited_);
            }
        }


        template <class t_Visitor>
        // cppcheck-suppress functionStatic
        void testProcessed()
        {
            ConfigurablePlain configurable;
            configurable.integer_ = 42;
            configurable.real_ = 1.5;

            {
                typename t_Visitor::Writer writer(getWriterInitializer("configurable_processed.cfg"));
                ariles2::apply(writer, configurable);
                BOOST_CHECK(configurable.prewrite_visited_);
            }

            {
                ConfigurablePlain read_configurable;

                typename t_Visitor::Reader reader(getReaderInitializer("configurable_processed.cfg"));
                ariles2::apply(reader, read_configurable);

                BOOST_CHECK_EQUAL(read_configurable.integer_, 42);
                BOOST_CHECK_CLOSE(read_configurable.real_, 1.5, g_tolerance);
                BOOST_CHECK(read_configurable.defaults_visited_);
                BOOST_CHECK(read_configurable.finalize_visited_);
            }
        }


    public:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            testPlain<t_Visitor>();
            testProcessed<t_Visitor>();
        }
    };
}  // namespace ariles_tests
