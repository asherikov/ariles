/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


#include "all_enabled_adapters.h"
#include <ariles2/ariles.h>


// ===============================================================
// TYPES
// ===============================================================

namespace testlib2
{
    /**
     * @brief Verbose definition of a configurable class (with explicit declaration
     * of members)
     */
    class ConfigurableVerbose : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_ENTRY_(v, integer)                                                                                         \
    ARILES2_ENTRY_(v, real)
#include ARILES2_INITIALIZE


    public:
        int integer_;
        double real_;


    public:
        ConfigurableVerbose()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        void arilesVisit(const ariles2::Defaults & /*visitor*/, const ariles2::Defaults::Parameters & /*param*/)
        {
            integer_ = 10;
            real_ = 1.33;
        }
    };
}  // namespace testlib2
