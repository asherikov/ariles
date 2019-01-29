/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.

    @brief
*/

#pragma once

#ifndef H_CPPUT_FLAGS
#define H_CPPUT_FLAGS

#include "exception.h"

namespace cpput
{
    template <class t_Derived>
    class Flags
    {
    public:
        enum Action
        {
            DEFAULT = 0,
            REPLACE = 1,
            SET = 2,
            UNSET = 3
        };


    public:
        unsigned int flags_;


    public:
        void initialize(const unsigned int flags, const Action action_type = REPLACE)
        {
            switch (action_type)
            {
                case REPLACE:
                    replace(flags);
                    break;

                case SET:
                    static_cast<t_Derived *>(this)->setDefaults();
                    set(flags);
                    break;

                case UNSET:
                    static_cast<t_Derived *>(this)->setDefaults();
                    unset(flags);
                    break;

                default:
                    CPPUT_THROW("Unknown Flags::Action type.");
            }
        }


        void copy(const t_Derived &from, const unsigned int mask)
        {
            set(from.flags_ & mask);
        }

        bool isSet(const unsigned int flags) const
        {
            return (flags_ & flags);
        }

        void replace(const unsigned int flags)
        {
            flags_ = flags;
        }

        void set(const unsigned int flags)
        {
            flags_ |= flags;
        }

        void unset(const unsigned int flags)
        {
            flags_ &= !flags;
        }
    };
}  // namespace cpput

#endif
