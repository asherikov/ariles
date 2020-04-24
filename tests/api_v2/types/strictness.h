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
    class ConfigurableStrictness1 : public ariles::DefaultBase
    {
#define ARILES_CONSTRUCTOR ConfigurableStrictness1
#define ARILES_ENTRIES ARILES_TYPED_ENTRY_(real, double)
#include ARILES_INITIALIZE


    public:
        ConfigurableStrictness1()
        {
            ariles::apply<ariles::Defaults>(*this);
        }


        /**
         * @brief This method must be defined
         */
        void arilesVisit(
                const ariles::Defaults & /*visitor*/,
                const ariles::Defaults::Parameters & /*param*/)
        {
            real_ = 1.33;
        }
    };


    class ConfigurableStrictness2 : public ConfigurableStrictness1
    {
#define ARILES_ENTRIES                                                                             \
    ARILES_TYPED_ENTRY_(integer, int)                                                              \
    ARILES_PARENT(ConfigurableStrictness1)
#include ARILES_INITIALIZE


    public:
        ConfigurableStrictness2()
        {
            ariles::apply<ariles::Defaults>(*this);
        }


        /**
         * @brief This method must be defined
         */
        void arilesVisit(const ariles::Defaults &visitor, const ariles::Defaults::Parameters &param)
        {
            integer_ = 10;
            ConfigurableStrictness1::arilesVisit(visitor, param);
        }
    };
}  // namespace ariles_tests
