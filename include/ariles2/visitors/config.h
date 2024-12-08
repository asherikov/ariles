/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "defaults.h"
#include "finalize.h"
#include "prewrite.h"
#include "read.h"
#include "write.h"
#include "aggregate.h"

/**
@defgroup config Configuration
@ingroup serialization

@brief Configuration visitors (perform pre- and post- processing during (de)serialization)
*/

namespace ariles2
{
    /// @ingroup config
    namespace cfgread
    {
        template <class t_Reader>
        class ARILES2_VISIBILITY_PUBLIC Visitor
          : public aggregate::Visitor<cfgread::Visitor<t_Reader>, Defaults, t_Reader, Finalize>
        {
        public:
            using AggregateBase = aggregate::Visitor<cfgread::Visitor<t_Reader>, Defaults, t_Reader, Finalize>;


        public:
            template <class... t_Initializers>
            explicit Visitor(t_Initializers &&...initializers)
              : AggregateBase(
                        std::tuple<>(),
                        std::forward_as_tuple(std::forward<t_Initializers>(initializers)...),
                        std::tuple<>())
            {
                CPPUT_TRACE_FUNCTION;
            }
        };
    }  // namespace cfgread
}  // namespace ariles2


namespace ariles2
{
    /// @ingroup config
    namespace cfgwrite
    {
        template <class t_Writer>
        class ARILES2_VISIBILITY_PUBLIC Visitor
          : public aggregate::Visitor<cfgwrite::Visitor<t_Writer>, PreWrite, t_Writer>
        {
        public:
            using AggregateBase = aggregate::Visitor<cfgwrite::Visitor<t_Writer>, PreWrite, t_Writer>;


        public:
            template <class... t_Initializers>
            explicit Visitor(t_Initializers &&...initializers)
              : AggregateBase(std::tuple<>(), std::forward_as_tuple(std::forward<t_Initializers>(initializers)...))
            {
                CPPUT_TRACE_FUNCTION;
            }
        };
    }  // namespace cfgwrite
}  // namespace ariles2
