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
    class ReadFixture : public t_FixtureBase
    {
    public:
        using t_FixtureBase::getReaderInitializer;


    protected:
        template <class t_Configurable, class t_Visitor>
        void test()
        {
            BOOST_CHECK_NO_THROW(t_Configurable configurable; ariles2::apply<typename t_Visitor::Reader>(
                                         getReaderInitializer("configurable.cfg"), configurable););
        }
    };
}  // namespace ariles_tests
