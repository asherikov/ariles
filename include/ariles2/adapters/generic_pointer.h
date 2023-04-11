/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/


namespace ariles2
{
    template <class t_Entry>
    bool isMissing(const ARILES2_POINTER_TYPE<t_Entry> &entry)
    {
        return (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(entry));
    }
}  // namespace ariles2


namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &reader,
                ARILES2_POINTER_TYPE<t_Entry> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            const bool is_null = reader.startPointer(parameters);
            if (is_null)
            {
                PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::reset(entry);
            }
            else
            {
                PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::allocate(entry);
                apply_read(reader, *entry, parameters);
            }
            reader.endPointer(is_null);
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

            const bool is_null = PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(entry);

            writer.startPointer(is_null, param);
            if (not is_null)
            {
                apply_write(writer, *entry, param);
            }
            writer.endPointer(is_null);
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
            if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(left))
            {
                if (not PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(right))
                {
                    visitor.equal_ = false;
                }
            }
            else
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(right))
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
            PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::reset(entry);
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
            if (not(PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(entry)))
            {
                apply_process(visitor, *entry, param);
            }
        }
    }  // namespace process
}  // namespace ariles2


namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                t_Visitor &visitor,
                ARILES2_POINTER_TYPE<t_Entry> &left,
                const ARILES2_POINTER_TYPE<t_Entry> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (param.deep_copy_)
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(right))
                {
                    PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::reset(left);
                }
                else
                {
                    if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(left))
                    {
                        PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::allocate(left);
                    }

                    apply_copyfrom(visitor, *left, *right, param);
                }
            }
            else
            {
                left = right;
            }
        }

        template <class t_Visitor, typename t_Left, typename t_Right>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                t_Visitor &visitor,
                ARILES2_POINTER_TYPE<t_Left> &left,
                const ARILES2_POINTER_TYPE<t_Right> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (param.deep_copy_)
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Right>>::isNull(right))
                {
                    PointerHandler<ARILES2_POINTER_TYPE<t_Left>>::reset(left);
                }
                else
                {
                    if (PointerHandler<ARILES2_POINTER_TYPE<t_Left>>::isNull(left))
                    {
                        PointerHandler<ARILES2_POINTER_TYPE<t_Left>>::allocate(left);
                    }

                    apply_copyfrom(visitor, *left, *right, param);
                }
            }
            else
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Right>>::isNull(right))
                {
                    PointerHandler<ARILES2_POINTER_TYPE<t_Left>>::reset(left);
                }
                else
                {
                    ARILES2_THROW("Shallow copies of pointers of different types are not supported.");
                }
            }
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, typename t_Entry>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor &visitor,
                const ARILES2_POINTER_TYPE<t_Entry> &left,
                ARILES2_POINTER_TYPE<t_Entry> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (param.deep_copy_)
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(left))
                {
                    PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::reset(right);
                }
                else
                {
                    if (PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::isNull(right))
                    {
                        PointerHandler<ARILES2_POINTER_TYPE<t_Entry>>::allocate(right);
                    }

                    apply_copyto(visitor, *left, *right, param);
                }
            }
            else
            {
                right = left;
            }
        }

        template <class t_Visitor, typename t_Left, typename t_Right>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor &visitor,
                const ARILES2_POINTER_TYPE<t_Left> &left,
                ARILES2_POINTER_TYPE<t_Right> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            if (param.deep_copy_)
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Left>>::isNull(left))
                {
                    PointerHandler<ARILES2_POINTER_TYPE<t_Right>>::reset(right);
                }
                else
                {
                    if (PointerHandler<ARILES2_POINTER_TYPE<t_Right>>::isNull(right))
                    {
                        PointerHandler<ARILES2_POINTER_TYPE<t_Right>>::allocate(right);
                    }

                    apply_copyto(visitor, *left, *right, param);
                }
            }
            else
            {
                if (PointerHandler<ARILES2_POINTER_TYPE<t_Left>>::isNull(left))
                {
                    PointerHandler<ARILES2_POINTER_TYPE<t_Right>>::reset(right);
                }
                else
                {
                    ARILES2_THROW("Shallow copies of pointers of different types are not supported.");
                }
            }
        }
    }  // namespace copyto
}  // namespace ariles2

#undef ARILES2_POINTER_HANDLER
#undef ARILES2_POINTER_TYPE
