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
    class CheckFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                t_Configurable  configurable_out;
                configurable_out.randomize();
                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Bridge::Writer>(getWriterInitializer("configurable_check.cfg"), configurable_out);
                );

                // -------

                t_Configurable  configurable_in;
                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Bridge::Reader>(getReaderInitializer("configurable_check.cfg"), configurable_in);
                );

                // -------

                check(configurable_in);
            }
    };
}
