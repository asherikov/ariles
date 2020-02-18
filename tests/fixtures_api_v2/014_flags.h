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
    template<class t_FixtureBase>
    class FlagsFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;
            using t_FixtureBase::getReaderInitializer;

        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                t_Configurable configurable;

                typename t_Bridge::Writer writer(getWriterInitializer("configurable.cfg"));
                BOOST_CHECK_EQUAL(configurable.getExpectedConfigurableFlags().flags_, configurable.arilesGetParameters(writer).flags_);
                ariles::apply(writer, configurable);

                typename t_Bridge::Reader reader(getReaderInitializer("configurable.cfg"));
                BOOST_CHECK_EQUAL(configurable.getExpectedConfigurableFlags().flags_, configurable.arilesGetParameters(reader).flags_);
            }
    };
}