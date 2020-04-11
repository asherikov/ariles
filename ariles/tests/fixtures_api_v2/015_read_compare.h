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
    class ReadCompareFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                t_Configurable configurable_default;
                t_Configurable configurable_read;

                configurable_default.setDefaults();

                BOOST_CHECK_NO_THROW(
                    ariles::apply<typename t_Bridge::Reader>(getReaderInitializer("configurable.cfg"), configurable_read);
                );

                compare(configurable_default, configurable_read);
            }
    };
}
