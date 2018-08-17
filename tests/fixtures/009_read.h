/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


template<class t_FixtureBase>
class ReadFixture : public t_FixtureBase
{
    public:
        using t_FixtureBase::getReaderInitializer;


    protected:
        template<class t_Configurable, class t_Bridge>
            void test()
        {
            BOOST_CHECK_NO_THROW(
                t_Configurable configurable;
                configurable.template readConfig<t_Bridge>(getReaderInitializer("configurable.cfg"));
            );
        }
};
