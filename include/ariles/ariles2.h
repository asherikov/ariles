/**
    @file
    @author  Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_API_VERSION 2

#ifndef ARILES_DISABLE
#    define ARILES_ENABLED
#endif


#include "internal/helpers.h"

#include "visitors/process.h"
#include "visitors/defaults.h"
#include "visitors/preprocess.h"
#include "visitors/postprocess.h"
#include "visitors/compare.h"
#include "visitors/count.h"
#include "visitors/read.h"
#include "visitors/write.h"
#include "visitors/config.h"

// These defines are always necessary
#define ARILES_TYPED_ENTRY_(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry##_, #entry)
#define ARILES_TYPED_ENTRY(entry, type) ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#include "base.h"

#ifdef ARILES_ENABLED
#    define ARILES_INITIALIZE "ariles/members/all.h"

#    define ARILES_ENTRY_(entry) ARILES_NAMED_ENTRY(entry##_, #    entry)
#    define ARILES_ENTRY(entry) ARILES_NAMED_ENTRY(entry, #    entry)


// ----------------------------

#        include "adapters/basic.h"
#        define ARILES_DEFAULT_VISITORS                                                                                \
            ARILES_VISITOR(count)                                                                                      \
            ARILES_VISITOR(postprocess)                                                                                \
            ARILES_VISITOR(preprocess)                                                                                 \
            ARILES_VISITOR(defaults)                                                                                   \
            ARILES_VISITOR(read)                                                                                       \
            ARILES_VISITOR(write)                                                                                      \
            ARILES_VISITOR(compare)

namespace ariles
{
    typedef Base<
            ariles::defaults::Base,
            ariles::postprocess::Base,
            ariles::preprocess::Base,
            ariles::count::Base,
            ariles::read::Base,
            ariles::write::Base>
            DefaultBase;
}

#else

#    define ARILES_DISABLED
#    define ARILES_INITIALIZE "ariles/members/variables.h"

namespace ariles
{
    // Some classes may inherit from this
    class ARILES_VISIBILITY_ATTRIBUTE DefaultBase
    {
    protected:
        /**
         * @brief Protected destructor: prevent destruction of derived
         * classes via base pointer.
         */
        ~DefaultBase()
        {
        }
        DefaultBase()
        {
        }

    public:
    };
}  // namespace ariles

#endif
