/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

namespace ariles
{
    namespace read
    {
        template <  class t_Visitor,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor &visitor,
                    ARILES_POINTER_TYPE<t_Entry> &entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            bool is_null = true;

            typename t_Visitor::Parameters param_local = param;
            param_local.unset(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES);

            visitor.template startMap<t_Visitor::SIZE_LIMIT_RANGE>(1, 2);
            applyToEntry(visitor, is_null, "is_null", param_local);

            if (true == is_null)
            {
                PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::reset(entry);
            }
            else
            {
                PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::allocate(entry);
                applyToEntry(visitor, *entry, "value", param_local);
            }
            visitor.endMap();
        }
    }
}


namespace ariles
{
    namespace write
    {
        template <  class t_Visitor,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            apply_write( t_Visitor & writer,
                   const ARILES_POINTER_TYPE<t_Entry> &entry,
                   const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            bool is_null = true;

            if (PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(entry))
            {
                is_null = true;
                writer.startMap(1);
                applyToEntry(writer, is_null, "is_null", param);
                writer.endMap();
            }
            else
            {
                is_null = false;
                writer.startMap(2);
                applyToEntry(writer, is_null, "is_null", param);
                applyToEntry(writer, *entry, "value", param);
                writer.endMap();
            }
        }
    }
}


namespace ariles
{
    namespace compare
    {
        template <class t_Visitor, typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const ARILES_POINTER_TYPE<t_Entry> &left,
                    const ARILES_POINTER_TYPE<t_Entry> &right,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (true == PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(left))
            {
                if (false == PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(right))
                {
                    visitor.equal_ = false;
                }
            }
            else
            {
                if (true == PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(right))
                {
                    visitor.equal_ = false;
                }
                else
                {
                    apply_compare(visitor, *left, *right, param);
                }
            }
        }
    }
}


namespace ariles
{
    namespace defaults
    {
        template <  class t_Visitor,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                    const t_Visitor & /*visitor*/,
                    ARILES_POINTER_TYPE<t_Entry> & entry,
                    const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::reset(entry);
        }
    }
}


namespace ariles
{
    namespace finalize
    {
        template <  class t_Visitor,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_finalize(
                    const t_Visitor & visitor,
                    ARILES_POINTER_TYPE<t_Entry> &entry,
                    const typename t_Visitor::Parameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (false == (PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(entry)))
            {
                apply_finalize(visitor, *entry, param);
            }
        }
    }
}


#undef ARILES_POINTER_HANDLER
#undef ARILES_POINTER_TYPE
