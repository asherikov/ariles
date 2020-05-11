/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles_tests
{
    class ConfigurableStrictness1 : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v) ARILES2_TYPED_ENTRY_(v, real, double)
#include ARILES2_INITIALIZE


    public:
        ConfigurableStrictness1()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        /**
         * @brief This method must be defined
         */
        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            real_ = 1.33;
        }
    };


    class ConfigurableStrictness2 : public ConfigurableStrictness1
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, integer, int)                                                                              \
    ARILES2_PARENT(v, ConfigurableStrictness1)
#include ARILES2_INITIALIZE


    public:
        ConfigurableStrictness2()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        /**
         * @brief This method must be defined
         */
        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            integer_ = 10;
            ConfigurableStrictness1::arilesVisit(visitor, param);
        }
    };
}  // namespace ariles_tests
