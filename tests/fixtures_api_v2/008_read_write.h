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
    class ReadWriteFixture : public t_FixtureBase
    {
        public:
            using t_FixtureBase::getWriterInitializer;
            using t_FixtureBase::getReaderInitializer;


        protected:
            template<class t_Configurable, class t_Bridge>
                void test()
            {
                BOOST_CHECK_NO_THROW(
                    t_Configurable configurable;
                    ariles::apply<typename t_Bridge::Reader>(getReaderInitializer("configurable2.cfg"), configurable);
                    ariles::apply<typename t_Bridge::Writer>(getWriterInitializer("configurable2.cfg"), configurable);
                );
            }
    };
}
