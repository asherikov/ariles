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
    namespace read
    {
        template<   class t_Iterator,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Iterator & iterator,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Iterator::ReadParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            if (iterator.descend(name))
            {
                try
                {
                    apply(iterator, entry, param);
                }
                catch(const std::exception &e)
                {
                    ARILES_THROW(std::string("Failed to parse entry <")
                                + name
                                + "> ||  "
                                + e.what());
                }

                iterator.ascend();
            }
            else
            {
                ARILES_PERSISTENT_ASSERT(   true == param.isSet(t_Iterator::ReadParameters::ALLOW_MISSING_ENTRIES),
                                            std::string("Configuration file does not contain entry '") + name + "'.");
            }
        }



        template <class t_Iterator, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & iterator,
                    t_Entry & entry,
                    const typename t_Iterator::ReadParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            ariles::count::Iterator counter;
            entry.arilesApply(counter);
            if (true == param.isSet(t_Iterator::ReadParameters::ALLOW_MISSING_ENTRIES))
            {
                iterator.template startMap<t_Iterator::SIZE_LIMIT_NONE>(counter.counter_);
            }
            else
            {
                iterator.template startMap<t_Iterator::SIZE_LIMIT_EQUAL>(counter.counter_);
            }
            entry.arilesApply(iterator, param);
            iterator.endMap();
        }


        /**
         * @brief Read configuration entry (an enum)
         * This function is necessary since an explicit casting to integer is needed.
         */
        template <  class t_Iterator,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & iterator,
                    t_Enumeration &entry,
                    const typename t_Iterator::ReadParameters & /*param*/,
                    // ENABLE this function for enums
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            int tmp_value = 0;
            iterator.readElement(tmp_value);
            entry = static_cast<t_Enumeration> (tmp_value);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Iterator> \
                    void ARILES_VISIBILITY_ATTRIBUTE apply( \
                                    t_Iterator & iterator, \
                                    type &entry, \
                                    const typename t_Iterator::ReadParameters & param) \
                { \
                    ARILES_TRACE_FUNCTION; \
                    ARILES_UNUSED_ARG(param); \
                    iterator.readElement(entry); \
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE
    }
}


namespace ariles
{
    template <class t_Writer>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                        t_Writer & writer,
                        const ariles::CommonConfigurableBase & entry,
                        const typename t_Writer::Parameters & param)
    {
        writer.startMap(entry.getNumberOfEntries());
        entry.writeConfigEntries(writer, param);
        writer.endMap();
    }



    template <  class t_Writer,
                typename t_Enumeration>
        void ARILES_VISIBILITY_ATTRIBUTE writeBody(
                        t_Writer & writer,
                        const t_Enumeration  entry,
                        const typename t_Writer::Parameters & /*param*/,
                        ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
    {
        int tmp_value = entry;
        writer.writeElement(tmp_value);
    }


    #define ARILES_BASIC_TYPE(type) \
            template <class t_Writer> \
                void ARILES_VISIBILITY_ATTRIBUTE writeBody( \
                                t_Writer &  writer, \
                                const type & entry, \
                                const typename t_Writer::Parameters & param) \
            {\
                ARILES_UNUSED_ARG(param); \
                writer.writeElement(entry); \
            }

    /**
     * @brief Generate writeBody methods for basic types.
     */
    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

    #undef ARILES_BASIC_TYPE



    template <  class t_Writer,
                typename t_Entry>
        void ARILES_VISIBILITY_ATTRIBUTE writeEntry(
                        t_Writer & writer,
                        const t_Entry & entry,
                        const std::string & entry_name,
                        const typename t_Writer::Parameters & param)
    {
        writer.descend(entry_name);
        writeBody(writer, entry, param);
        writer.ascend();
    }
}


namespace ariles
{
    namespace compare
    {
        template<   class t_Iterator,
                    class t_Left,
                    class t_Right>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Iterator & iterator,
                    const t_Left & left,
                    const t_Right & right,
                    const std::string & name,
                    const typename t_Iterator::CompareParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(left);
            ARILES_TRACE_TYPE(right);

            try
            {
                if (false == apply(iterator, left, right, param))
                {
                    ARILES_THROW("");
                }
            }
            catch (const std::exception & e)
            {
                ARILES_THROW("entry: " + name + " // " + std::string(e.what()));
            }
        }


        template<   class t_Iterator,
                    class t_Left,
                    class t_Right>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Iterator & /*iterator*/,
                    const t_Left & left,
                    const t_Right & right,
                    const typename t_Iterator::CompareParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Left) * = NULL)
        {
            return (left.arilesCompare(right, param));
        }


        template <  class t_Iterator,
                    typename t_Enumeration>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    const t_Enumeration & left,
                    const t_Enumeration & right,
                    const typename t_Iterator::CompareParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            return (left == right);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Iterator> \
                    inline bool ARILES_VISIBILITY_ATTRIBUTE apply( \
                            const t_Iterator &, \
                            const type & left, \
                            const type & right, \
                            const typename t_Iterator::CompareParameters &) \
                { return (left == right); }

        /**
         * @brief Generate compare methods for basic types.
         */
        #define ARILES_COMPARE_TYPES_LIST \
                ARILES_BASIC_INTEGER_TYPES_LIST \
                ARILES_BASIC_TYPE(bool) \
                ARILES_BASIC_TYPE(std::string)

        ARILES_MACRO_SUBSTITUTE(ARILES_COMPARE_TYPES_LIST)

        #undef ARILES_BASIC_TYPE



        template<class t_Iterator>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    const float & left,
                    const float & right,
                    const typename t_Iterator::CompareParameters & param)
        {
            return (iterator.compareFloats(left, right, param));
        }


        template<class t_Iterator>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    const double & left,
                    const double & right,
                    const typename t_Iterator::CompareParameters & param)
        {
            return (iterator.compareFloats(left, right, param));
        }
    }
}


namespace ariles
{
    namespace defaults
    {
        template<   class t_Iterator,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    const t_Iterator & iterator,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Iterator::DefaultsParameters & param)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            apply(iterator, entry, param);
        }


        template<class t_Iterator, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & iterator,
                    t_Entry & entry,
                    const typename t_Iterator::DefaultsParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry.arilesApply(iterator, param);
        }


        template <  class t_Iterator,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    t_Enumeration & entry,
                    const typename t_Iterator::DefaultsParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry = static_cast<t_Enumeration>(0);
        }


        #define ARILES_BASIC_TYPE(type) \
            template<class t_Iterator> \
                void ARILES_VISIBILITY_ATTRIBUTE apply( \
                        const t_Iterator &, \
                        type & entry, \
                        const typename t_Iterator::DefaultsParameters & param) \
                { \
                    ARILES_TRACE_FUNCTION; \
                    entry = param.template getDefault<type>(); \
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE
    }
}



namespace ariles
{
    namespace finalize
    {
        template<   class t_Iterator,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    const t_Iterator & iterator,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Iterator::FinalizeParameters & param)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            apply(iterator, entry, param);
        }


        template<class t_Iterator, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                        const t_Iterator & iterator,
                        t_Entry & entry,
                        const typename t_Iterator::FinalizeParameters & param,
                        ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry.arilesApply(iterator, param);
        }


        template <  class t_Iterator,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Iterator & /*iterator*/,
                    t_Enumeration & /*entry*/,
                    const typename t_Iterator::FinalizeParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
        }


        #define ARILES_BASIC_TYPE(type) \
            template<class t_Iterator> \
                void ARILES_VISIBILITY_ATTRIBUTE apply( \
                        const t_Iterator &, \
                        const type &, \
                        const typename t_Iterator::FinalizeParameters &) \
                { \
                    ARILES_TRACE_FUNCTION; \
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE
    }
}


namespace ariles
{
    namespace count
    {
        template<   class t_Iterator,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Iterator & iterator,
                    const t_Entry & entry,
                    const std::string & name,
                    const typename t_Iterator::CountParameters & /*param*/,
                    ARILES_IS_CONFIGURABLE_DISABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_UNUSED_ARG(entry);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            ++iterator.counter_;
        }


        template<   class t_Iterator,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Iterator & iterator,
                    const t_Entry & entry,
                    const std::string & name,
                    const typename t_Iterator::CountParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            if (true == iterator.descend_)
            {
                iterator.descend_ = false;
                entry.arilesApply(iterator, param);
            }
            else
            {
                ++iterator.counter_;
            }
        }
    }
}
