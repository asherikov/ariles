/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    class BridgeFlags : public ariles::Flags<unsigned int, BridgeFlags>
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
            BridgeFlags()
            {
                setDefaults();
            }

            BridgeFlags(const unsigned int flags, const Action action_type = REPLACE)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
                flags_ = DEFAULT;
            }
    };
}
