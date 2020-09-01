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
    class FlagsFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;
        using t_FixtureBase::getWriterInitializer;

    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            t_Configurable configurable;

            typename t_Visitor::Writer writer(getWriterInitializer("configurable.cfg"));
            BOOST_CHECK_EQUAL(
                    configurable.getExpectedWriteParameters().sloppy_maps_,
                    configurable.arilesGetParameters(writer.getWriter()).sloppy_maps_);
            BOOST_CHECK_EQUAL(
                    configurable.getExpectedWriteParameters().sloppy_pairs_,
                    configurable.arilesGetParameters(writer.getWriter()).sloppy_pairs_);
            ariles2::apply(writer, configurable);

            typename t_Visitor::Reader reader(getReaderInitializer("configurable.cfg"));
            BOOST_CHECK_EQUAL(
                    configurable.getExpectedReadParameters().sloppy_maps_,
                    configurable.arilesGetParameters(reader.getReader()).sloppy_maps_);
            BOOST_CHECK_EQUAL(
                    configurable.getExpectedReadParameters().sloppy_pairs_,
                    configurable.arilesGetParameters(reader.getReader()).sloppy_pairs_);
        }
    };
}  // namespace ariles_tests
