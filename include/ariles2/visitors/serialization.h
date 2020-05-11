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

namespace ariles2
{
    namespace serialization
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Features : public ariles2::Flags<unsigned int, serialization::Features>
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


        class ARILES2_VISIBILITY_ATTRIBUTE Base
          : public visitor::Base<visitor::GenericVisitor, ariles2::ConfigurableFlags>
        {
        public:
            typedef ariles2::ConfigurableFlags Parameters;

        public:
            using visitor::Base<visitor::GenericVisitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            virtual const Features &getSerializationFeatures() const = 0;
        };
    }  // namespace serialization
}  // namespace ariles2
