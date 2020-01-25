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
        template <  class t_Iterator,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator &iterator,
                    ARILES_POINTER_TYPE<t_Entry> &entry,
                    const typename t_Iterator::ReadParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            bool is_null = true;

            typename t_Iterator::ReadParameters param_local = param;
            param_local.unset(t_Iterator::ReadParameters::ALLOW_MISSING_ENTRIES);

            iterator.template startMap<t_Iterator::SIZE_LIMIT_RANGE>(1, 2);
            arilesEntryApply(iterator, is_null, "is_null", param_local);

            if (true == is_null)
            {
                PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::reset(entry);
            }
            else
            {
                PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::allocate(entry);
                arilesEntryApply(iterator, *entry, "value", param_local);
            }
            iterator.endMap();
        }
    }
}


namespace ariles
{
    template <  class t_Writer,
                typename t_Entry>
        void ARILES_VISIBILITY_ATTRIBUTE
        writeBody( t_Writer & writer,
                   const ARILES_POINTER_TYPE<t_Entry> &entry,
                   const typename t_Writer::Parameters & param)
    {
        bool is_null = true;

        if (PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(entry))
        {
            is_null = true;
            writer.startMap(1);
            writeEntry(writer, is_null, "is_null", param);
            writer.endMap();
        }
        else
        {
            is_null = false;
            writer.startMap(2);
            writeEntry(writer, is_null, "is_null", param);
            writeEntry(writer, *entry, "value", param);
            writer.endMap();
        }
    }
}


namespace ariles
{
    namespace compare
    {
        template <class t_Iterator, typename t_Entry>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    const ARILES_POINTER_TYPE<t_Entry> &left,
                    const ARILES_POINTER_TYPE<t_Entry> &right,
                    const typename t_Iterator::CompareParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (true == PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(left))
            {
                if (true == PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(right))
                {
                    return (true);
                }
                else
                {
                    return (false);
                }
            }
            else
            {
                if (true == PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(right))
                {
                    return (false);
                }
                else
                {
                    return (apply(iterator, *left, *right, param));
                }
            }
        }
    }
}


namespace ariles
{
    namespace defaults
    {
        template <  class t_Iterator,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    ARILES_POINTER_TYPE<t_Entry> & entry,
                    const typename t_Iterator::DefaultsParameters & /*param*/)
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
        template <  class t_Iterator,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    ARILES_POINTER_TYPE<t_Entry> &entry,
                    const typename t_Iterator::FinalizeParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            if (false == (PointerHandler<ARILES_POINTER_TYPE<t_Entry> >::isNull(entry)))
            {
                apply(iterator, *entry, param);
            }
        }
    }
}


#undef ARILES_POINTER_HANDLER
#undef ARILES_POINTER_TYPE
