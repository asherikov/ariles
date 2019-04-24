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
    class DummyFixture
    {
        protected:
            template<class t_Configurable>
                void test()
            {
                BOOST_CHECK_NO_THROW(
                    t_Configurable configurable;
                    configurable.setDefaults();
                );
            }
    };
}
