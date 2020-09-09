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
    /**
     * @brief Configurable class without extra constructors.
     */
    class ConfigurableEmpty : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)
#include ARILES2_INITIALIZE


    public:
        ConfigurableEmpty()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        /**
         * @brief This method must be defined
         */
        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
        }


        void randomize()
        {
            ariles2::apply<ariles2::PostRead>(*this);
        }
    };
}  // namespace ariles_tests
