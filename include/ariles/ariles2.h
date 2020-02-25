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

// must be first
#ifndef ARILES_API_VERSION
#   define ARILES_API_VERSION 2
#endif

#ifndef ARILES_DISABLE
#   define ARILES_ENABLED
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
#define ARILES_TYPED_ENTRY(entry, type)  ARILES_TYPED_NAMED_ENTRY(type, entry, #entry)


#ifdef ARILES_ENABLED
    #define ARILES_INITIALIZE  "ariles/members/all.h"

    #define ARILES_ENTRY_(entry)     ARILES_NAMED_ENTRY(entry##_, #entry)
    #define ARILES_ENTRY(entry)      ARILES_NAMED_ENTRY(entry, #entry)


    #define ARILES_DEFAULT_VISITORS \
        ARILES_VISITOR(count) \
        ARILES_VISITOR(postprocess) \
        ARILES_VISITOR(preprocess) \
        ARILES_VISITOR(defaults) \
        ARILES_VISITOR(read) \
        ARILES_VISITOR(write) \
        ARILES_VISITOR(compare)


    // ----------------------------


    namespace ariles
    {
        class ARILES_VISIBILITY_ATTRIBUTE Ariles
        {
            protected:
                ~Ariles() {}
                Ariles() {}
        };


        /// @todo variadic template
        class ARILES_VISIBILITY_ATTRIBUTE DefaultBase
            :   public ariles::Ariles,
                public ariles::defaults::Base,
                public ariles::postprocess::Base,
                public ariles::preprocess::Base,
                public ariles::count::Base,
                public ariles::read::Base,
                public ariles::write::Base
        {
            protected:
                /**
                 * @brief Protected destructor: prevent destruction of the child
                 * classes through a base pointer.
                 */
                ~DefaultBase() {}
                DefaultBase() {}


            public:
                using ariles::defaults::Base::arilesVirtualVisit;
                using ariles::postprocess::Base::arilesVirtualVisit;
                using ariles::preprocess::Base::arilesVirtualVisit;
                using ariles::read::Base::arilesVirtualVisit;
                using ariles::write::Base::arilesVirtualVisit;
                using ariles::count::Base::arilesVirtualVisit;


                using ariles::defaults::Base::arilesGetParameters;
                using ariles::postprocess::Base::arilesGetParameters;
                using ariles::preprocess::Base::arilesGetParameters;
                using ariles::read::Base::arilesGetParameters;
                using ariles::write::Base::arilesGetParameters;
                using ariles::count::Base::arilesGetParameters;


                virtual const std::string & arilesDefaultID() const = 0;
        };
    }

#   if 2 == ARILES_API_VERSION
#       include "adapters/basic.h"
#   endif

#else

#   define ARILES_DISABLED
#   define ARILES_INITIALIZE  "ariles/members/variables.h"

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
                ~DefaultBase() {}
                DefaultBase() {}

            public:
        };
    }

#endif
