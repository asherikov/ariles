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

#ifndef ARILES2_DISABLE
#    define ARILES2_ENABLED
#endif


#include "internal/helpers.h"

#include "visitors/process.h"
#include "visitors/defaults.h"
#include "visitors/preprocess.h"
#include "visitors/postprocess.h"
#include "visitors/count.h"
#include "visitors/read.h"
#include "visitors/write.h"
#include "visitors/config.h"

// These defines are always necessary
#define ARILES2_TYPED_ENTRY_(v, entry, type) ARILES2_TYPED_NAMED_ENTRY(v, type, entry##_, entry)
#define ARILES2_TYPED_ENTRY(v, entry, type) ARILES2_TYPED_NAMED_ENTRY(v, type, entry, entry)


#ifdef ARILES2_ENABLED
#    define ARILES2_INITIALIZE <ariles2/members/all.h>

#    define ARILES2_PARENT(v, entry) ARILES2_PARENT_##v(v, entry)
#    define ARILES2_NAMED_ENTRY(v, entry, name) ARILES2_NAMED_ENTRY_##v(v, entry, name)

#    define ARILES2_ENTRY_(v, entry) ARILES2_NAMED_ENTRY(v, entry##_, entry)
#    define ARILES2_ENTRY(v, entry) ARILES2_NAMED_ENTRY(v, entry, entry)

// ----------------------------

#    include "adapters/basic.h"

#    ifndef ARILES2_DEFAULT_VISITORS
#        define ARILES2_DEFAULT_VISITORS                                                                               \
            ARILES2_VISITOR(count)                                                                                     \
            ARILES2_VISITOR(postprocess)                                                                               \
            ARILES2_VISITOR(preprocess)                                                                                \
            ARILES2_VISITOR(defaults)                                                                                  \
            ARILES2_VISITOR(read)                                                                                      \
            ARILES2_VISITOR(write)
#    endif

namespace ariles2
{
    class ARILES2_VISIBILITY_ATTRIBUTE DefaultBase : public ariles2::Ariles
#    define ARILES2_VISITOR(visitor) , public ariles2::visitor::Base
                                                             ARILES2_DEFAULT_VISITORS
#    undef ARILES2_VISITOR
    {
    protected:
        DefaultBase()
        {
        }
        ~DefaultBase()
        {
        }

    public:
#    define ARILES2_VISITOR(visitor) ARILES2_BASE_METHODS_##visitor
        ARILES2_DEFAULT_VISITORS
#    undef ARILES2_VISITOR
    };
}  // namespace ariles2

#else

#    define ARILES2_DISABLED
#    define ARILES2_INITIALIZE <ariles2/members/variables.h>

namespace ariles2
{
    // Some classes may inherit from this
    class ARILES2_VISIBILITY_ATTRIBUTE DefaultBase
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
}  // namespace ariles2

#endif
