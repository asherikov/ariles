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


    // ----------------------------


    namespace ariles
    {
        class ARILES_VISIBILITY_ATTRIBUTE Ariles
        {
            protected:
                ~Ariles() {}
                Ariles() {}

            public:
                virtual const std::string & arilesDefaultID() const = 0;
        };



#       define  ARILES_USE_BASE(Base) \
                using Base::arilesVirtualVisit; \
                using Base::arilesGetParameters;

        // 10
        template<   class t_B0       , class t_B1 = void, class t_B2 = void, class t_B3 = void, class t_B4 = void,
                    class t_B5 = void, class t_B6 = void, class t_B7 = void, class t_B8 = void, class t_B9 = void>
            class ARILES_VISIBILITY_ATTRIBUTE Base
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3, public t_B4,
                public t_B5, public t_B6, public t_B7, public t_B8, public t_B9
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
                ARILES_USE_BASE(t_B4)
                ARILES_USE_BASE(t_B5)
                ARILES_USE_BASE(t_B6)
                ARILES_USE_BASE(t_B7)
                ARILES_USE_BASE(t_B8)
                ARILES_USE_BASE(t_B9)
        };

        // 9
        template<   class t_B0, class t_B1, class t_B2, class t_B3, class t_B4,
                    class t_B5, class t_B6, class t_B7, class t_B8>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2, t_B3, t_B4, t_B5, t_B6, t_B7, t_B8>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3, public t_B4,
                public t_B5, public t_B6, public t_B7, public t_B8
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
                ARILES_USE_BASE(t_B4)
                ARILES_USE_BASE(t_B5)
                ARILES_USE_BASE(t_B6)
                ARILES_USE_BASE(t_B7)
                ARILES_USE_BASE(t_B8)
        };

        // 8
        template<   class t_B0, class t_B1, class t_B2, class t_B3, class t_B4,
                    class t_B5, class t_B6, class t_B7>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2, t_B3, t_B4, t_B5, t_B6, t_B7>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3, public t_B4,
                public t_B5, public t_B6, public t_B7
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
                ARILES_USE_BASE(t_B4)
                ARILES_USE_BASE(t_B5)
                ARILES_USE_BASE(t_B6)
                ARILES_USE_BASE(t_B7)
        };

        // 7
        template<   class t_B0, class t_B1, class t_B2, class t_B3, class t_B4,
                    class t_B5, class t_B6>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2, t_B3, t_B4, t_B5, t_B6>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3, public t_B4,
                public t_B5, public t_B6
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
                ARILES_USE_BASE(t_B4)
                ARILES_USE_BASE(t_B5)
                ARILES_USE_BASE(t_B6)
        };

        // 6
        template<   class t_B0, class t_B1, class t_B2, class t_B3, class t_B4,
                    class t_B5>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2, t_B3, t_B4, t_B5>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3, public t_B4,
                public t_B5
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
                ARILES_USE_BASE(t_B4)
                ARILES_USE_BASE(t_B5)
        };

        // 5
        template<class t_B0, class t_B1, class t_B2, class t_B3, class t_B4>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2, t_B3, t_B4>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3, public t_B4
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
                ARILES_USE_BASE(t_B4)
        };

        // 4
        template<class t_B0, class t_B1, class t_B2, class t_B3>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2, t_B3>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2, public t_B3
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
                ARILES_USE_BASE(t_B3)
        };

        // 3
        template<class t_B0, class t_B1, class t_B2>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1, t_B2>
            :   public ariles::Ariles,
                public t_B0, public t_B1, public t_B2
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
                ARILES_USE_BASE(t_B2)
        };

        // 2
        template<class t_B0, class t_B1>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0, t_B1>
            :   public ariles::Ariles,
                public t_B0, public t_B1
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
                ARILES_USE_BASE(t_B1)
        };

        // 1
        template<class t_B0>
            class ARILES_VISIBILITY_ATTRIBUTE Base<t_B0>
            :   public ariles::Ariles,
                public t_B0
        {
            protected:
                Base(){}
                ~Base(){}
            public:
                ARILES_USE_BASE(t_B0)
        };

#       undef  ARILES_USE_BASE


#       define ARILES_DEFAULT_VISITORS \
                ARILES_VISITOR(count) \
                ARILES_VISITOR(postprocess) \
                ARILES_VISITOR(preprocess) \
                ARILES_VISITOR(defaults) \
                ARILES_VISITOR(read) \
                ARILES_VISITOR(write) \
                ARILES_VISITOR(compare)


        typedef Base<   ariles::defaults::Base,
                        ariles::postprocess::Base,
                        ariles::preprocess::Base,
                        ariles::count::Base,
                        ariles::read::Base,
                        ariles::write::Base> DefaultBase;
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
