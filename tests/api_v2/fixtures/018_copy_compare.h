/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class CopyCompareFixture
    {
    protected:
        template <class t_Configurable, class t_Other>
        void test()
        {
            t_Configurable configurable1, configurable2;
            t_Other configurable_copy;

            configurable1.randomize();

            ariles2::apply<ariles2::CopyTo>(configurable1, configurable_copy);
            ariles2::apply<ariles2::CopyFrom>(configurable2, configurable_copy);

            BOOST_CHECK(true == ariles2::apply<ariles2::Compare>(configurable1, configurable2));
        }
    };
}  // namespace ariles_tests
