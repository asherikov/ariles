/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "../internal/helpers.h"
#include "../visitors/count.h"

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, class t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                t_Entry &entry,
                const typename t_Visitor::Parameters &parameters,
                ARILES2_IS_BASE_ENABLER(ariles2::read::Base, t_Entry))
        {
            ARILES2_TRACE_FUNCTION;

            if (true == parameters.override_parameters_)
            {
                visitor.startMap(entry, parameters);
                entry.arilesVirtualVisit(visitor, parameters);
            }
            else
            {
                visitor.startMap(entry, entry.arilesGetParameters(visitor));
                entry.arilesVirtualVisit(visitor, entry.arilesGetParameters(visitor));
            }
            visitor.endMap();
        }


        /**
         * @brief Read configuration entry (an enum)
         * This function is necessary since an explicit casting to integer is needed.
         */
        template <class t_Visitor, typename t_Enumeration>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                t_Enumeration &entry,
                const typename t_Visitor::Parameters & /*param*/,
                // ENABLE this function for enums
                ARILES2_IS_ENUM_ENABLER(t_Enumeration))
        {
            ARILES2_TRACE_FUNCTION;
            int tmp_value = 0;
            visitor.readElement(tmp_value);
            entry = static_cast<t_Enumeration>(tmp_value);
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    template <class t_Visitor>                                                                                         \
    void ARILES2_VISIBILITY_ATTRIBUTE apply_read(                                                                      \
            t_Visitor &visitor, type &entry, const typename t_Visitor::Parameters &param)                              \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_UNUSED_ARG(param);                                                                                     \
        visitor.readElement(entry);                                                                                    \
    }

        ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, class t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const t_Entry &entry,
                const typename t_Visitor::Parameters &parameters,
                ARILES2_IS_BASE_ENABLER(ariles2::write::Base, t_Entry))
        {
            ARILES2_TRACE_FUNCTION;

            if (true == parameters.override_parameters_)
            {
                writer.startMap(entry, parameters);
                entry.arilesVirtualVisit(writer, parameters);
            }
            else
            {
                writer.startMap(entry, entry.arilesGetParameters(writer));
                entry.arilesVirtualVisit(writer, entry.arilesGetParameters(writer));
            }
            writer.endMap();
        }



        template <class t_Visitor, typename t_Enumeration>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const t_Enumeration entry,
                const typename t_Visitor::Parameters &param,
                ARILES2_IS_ENUM_ENABLER(t_Enumeration))
        {
            ARILES2_TRACE_FUNCTION;
            int tmp_value = entry;
            writer.writeElement(tmp_value, param);
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    template <class t_Visitor>                                                                                         \
    void ARILES2_VISIBILITY_ATTRIBUTE apply_write(                                                                     \
            t_Visitor &writer, const type &entry, const typename t_Visitor::Parameters &param)                         \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        writer.writeElement(entry, param);                                                                             \
    }

        /**
         * @brief Generate apply methods for basic types.
         */
        ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, class t_Left, class t_Right>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const t_Left &left,
                const t_Right &right,
                const typename t_Visitor::Parameters &param,
                ARILES2_IS_BASE_ENABLER(ariles2::Ariles, t_Left))
        {
            ARILES2_TRACE_FUNCTION;
            if (true == param.compare_number_of_entries_)
            {
                visitor.equal_ &= (ariles2::count::Visitor().count(left) == ariles2::count::Visitor().count(right));
            }
            left.arilesVisit(visitor, right, param);
        }


        template <class t_Visitor, typename t_Enumeration>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const t_Enumeration &left,
                const t_Enumeration &right,
                const typename t_Visitor::Parameters & /*param*/,
                ARILES2_IS_ENUM_ENABLER(t_Enumeration))
        {
            visitor.equal_ &= (left == right);
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    template <class t_Visitor>                                                                                         \
    inline void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(                                                            \
            t_Visitor &visitor, const type &left, const type &right, const typename t_Visitor::Parameters &)           \
    {                                                                                                                  \
        visitor.equal_ &= (left == right);                                                                             \
    }

/**
 * @brief Generate compare methods for basic types.
 */
#define ARILES2_COMPARE_TYPES_LIST                                                                                     \
    ARILES2_BASIC_INTEGER_TYPES_LIST                                                                                   \
    ARILES2_BASIC_TYPE(bool)                                                                                           \
    ARILES2_BASIC_TYPE(std::string)

        ARILES2_COMPARE_TYPES_LIST

#undef ARILES2_BASIC_TYPE



        template <class t_Visitor>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const float &left,
                const float &right,
                const typename t_Visitor::Parameters &param)
        {
            visitor.equal_ &= (visitor.compareFloats(left, right, param));
        }


        template <class t_Visitor>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const double &left,
                const double &right,
                const typename t_Visitor::Parameters &param)
        {
            visitor.equal_ &= visitor.compareFloats(left, right, param);
        }
    }  // namespace compare
}  // namespace ariles2


namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, class t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor &visitor,
                t_Entry &entry,
                const typename t_Visitor::Parameters &param,
                ARILES2_IS_BASE_ENABLER(ariles2::defaults::Base, t_Entry))
        {
            ARILES2_TRACE_FUNCTION;
            entry.arilesVirtualVisit(visitor, param);
        }


        template <class t_Visitor, typename t_Enumeration>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                t_Enumeration & /*entry*/,
                const typename t_Visitor::Parameters & /*param*/,
                ARILES2_IS_ENUM_ENABLER(t_Enumeration))
        {
            ARILES2_TRACE_FUNCTION;
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    template <class t_Visitor>                                                                                         \
    void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(                                                                  \
            const t_Visitor &, type &entry, const typename t_Visitor::Parameters &param)                               \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        entry = param.template getDefault<type>();                                                                     \
    }

        ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE
    }  // namespace defaults
}  // namespace ariles2



namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, class t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &visitor,
                t_Entry &entry,
                const typename t_Visitor::Parameters &param,
                ARILES2_IS_BASE_ENABLER(ariles2::Ariles, t_Entry))
        {
            ARILES2_TRACE_FUNCTION;
            entry.arilesVirtualVisit(visitor, param);
        }


        template <class t_Visitor, class t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &,
                t_Entry &,
                const typename t_Visitor::Parameters &,
                ARILES2_IS_BASE_DISABLER(ariles2::Ariles, t_Entry))
        {
            ARILES2_TRACE_FUNCTION;
        }
    }  // namespace process
}  // namespace ariles2
