/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

#include "../configurable_flags.h"

namespace ariles
{
    namespace serialization
    {
        class ARILES_VISIBILITY_ATTRIBUTE Features
          : public ariles::Flags<unsigned int, serialization::Features>
        {
        public:
            enum Flags
            {
                RESET = 0,
                SLOPPY_MAPS_SUPPORTED = 1,
                NATIVE_MATRIX_SUPPORTED = 2,
                SLOPPY_PAIRS_SUPPORTED = 4,

                DEFAULT = RESET
            };


        public:
            Features()
            {
                setDefaults();
            }

            Features(const unsigned int flags, const Action action_type = REPLACE)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
                flags_ = DEFAULT;
            }
        };


        class ARILES_VISIBILITY_ATTRIBUTE Base : public visitor::Base<ariles::ConfigurableFlags>
        {
        public:
            typedef ariles::ConfigurableFlags Parameters;

        public:
            using visitor::Base<Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            virtual const Features &getSerializationFeatures() const = 0;
#if 1 == ARILES_API_VERSION
            const Features &getBridgeFlags() const
            {
                return (getSerializationFeatures());
            }
#endif
        };
    }  // namespace serialization
}  // namespace ariles
