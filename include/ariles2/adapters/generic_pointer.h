/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                ARILES2_POINTER_TYPE<t_Entry> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            bool is_null = true;

            visitor.template startMap<t_Visitor::SIZE_LIMIT_RANGE>(1, 2);
            visitor.override_missing_entries_locally_ = true;
            visitor(is_null, "is_null", parameters);

            if (true == is_null)
            {
                PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::reset(entry);
            }
            else
            {
                PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::allocate(entry);
                visitor.override_missing_entries_locally_ = true;
                visitor(*entry, "value", parameters);
            }
            visitor.endMap();
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const ARILES2_POINTER_TYPE<t_Entry> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            bool is_null = true;

            if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::isNull(entry))
            {
                is_null = true;
                writer.startMap("", 1);
                writer(is_null, "is_null", param);
                writer.endMap();
            }
            else
            {
                is_null = false;
                writer.startMap("", 2);
                writer(is_null, "is_null", param);
                writer(*entry, "value", param);
                writer.endMap();
            }
        }
    }  // namespace write
}  // namespace ariles2


namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const ARILES2_POINTER_TYPE<t_Entry> &left,
                const ARILES2_POINTER_TYPE<t_Entry> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::isNull(left))
            {
                if (false == PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::isNull(right))
                {
                    visitor.equal_ = false;
                }
            }
            else
            {
                if (true == PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::isNull(right))
                {
                    visitor.equal_ = false;
                }
                else
                {
                    apply_compare(visitor, *left, *right, param);
                }
            }
        }
    }  // namespace compare
}  // namespace ariles2


namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                ARILES2_POINTER_TYPE<t_Entry> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::reset(entry);
        }
    }  // namespace defaults
}  // namespace ariles2


namespace ariles2
{
    namespace process
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_process(
                const t_Visitor &visitor,
                ARILES2_POINTER_TYPE<t_Entry> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (false == (PointerHandler<ARILES2_POINTER_TYPE<t_Entry> >::isNull(entry)))
            {
                apply_process(visitor, *entry, param);
            }
        }
    }  // namespace process
}  // namespace ariles2


#undef ARILES2_POINTER_HANDLER
#undef ARILES2_POINTER_TYPE
