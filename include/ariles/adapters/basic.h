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
        template<   class t_Visitor,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Visitor & visitor,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::ReadParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            if (visitor.descend(name))
            {
                try
                {
                    apply(visitor, entry, param);
                }
                catch(const std::exception &e)
                {
                    ARILES_THROW(std::string("Failed to parse entry <")
                                + name
                                + "> ||  "
                                + e.what());
                }

                visitor.ascend();
            }
            else
            {
                ARILES_PERSISTENT_ASSERT(   true == param.isSet(t_Visitor::ReadParameters::ALLOW_MISSING_ENTRIES),
                                            std::string("Configuration file does not contain entry '") + name + "'.");
            }
        }



        template <class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Visitor & visitor,
                    t_Entry & entry,
                    const typename t_Visitor::ReadParameters & parameters,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;

            typename t_Visitor::ReadParameters param = parameters;
            if (false == param.isSet(t_Visitor::ReadParameters::PROPAGATE_ALLOW_MISSING_ENTRIES))
            {
                param.set(t_Visitor::ReadParameters::DEFAULT & t_Visitor::ReadParameters::ALLOW_MISSING_ENTRIES);
            }

            ariles::count::Visitor counter;
            entry.ariles(counter);
            if (true == param.isSet(t_Visitor::ReadParameters::ALLOW_MISSING_ENTRIES))
            {
                visitor.template startMap<t_Visitor::SIZE_LIMIT_NONE>(counter.counter_);
            }
            else
            {
                visitor.template startMap<t_Visitor::SIZE_LIMIT_EQUAL>(counter.counter_);
            }
            entry.arilesApply(visitor, param);
            visitor.endMap();

            entry.finalize(); /// @todo DEPRECATED
        }


        /**
         * @brief Read configuration entry (an enum)
         * This function is necessary since an explicit casting to integer is needed.
         */
        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Visitor & visitor,
                    t_Enumeration &entry,
                    const typename t_Visitor::ReadParameters & /*param*/,
                    // ENABLE this function for enums
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            int tmp_value = 0;
            visitor.readElement(tmp_value);
            entry = static_cast<t_Enumeration> (tmp_value);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Visitor> \
                    void ARILES_VISIBILITY_ATTRIBUTE apply( \
                                    t_Visitor & visitor, \
                                    type &entry, \
                                    const typename t_Visitor::ReadParameters & param) \
                { \
                    ARILES_TRACE_FUNCTION; \
                    ARILES_UNUSED_ARG(param); \
                    visitor.readElement(entry); \
                }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE
    }
}


namespace ariles
{
    namespace write
    {
        template <  class t_Visitor,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Visitor & writer,
                    const t_Entry & entry,
                    const std::string & entry_name,
                    const typename t_Visitor::WriteParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);

            writer.descend(entry_name);
            apply(writer, entry, param);
            writer.ascend();
        }



        template <class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Visitor & writer,
                    const t_Entry & entry,
                    const typename t_Visitor::WriteParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            ariles::count::Visitor counter;
            entry.ariles(counter);
            writer.startMap(counter.counter_);
            entry.arilesApply(writer, param);
            writer.endMap();
        }



        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Visitor & writer,
                    const t_Enumeration entry,
                    const typename t_Visitor::WriteParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            int tmp_value = entry;
            writer.writeElement(tmp_value);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Visitor> \
                    void ARILES_VISIBILITY_ATTRIBUTE apply( \
                            t_Visitor &  writer, \
                            const type & entry, \
                            const typename t_Visitor::WriteParameters &) \
                {\
                    ARILES_TRACE_FUNCTION; \
                    writer.writeElement(entry); \
                }

        /**
         * @brief Generate apply methods for basic types.
         */
        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

        #undef ARILES_BASIC_TYPE
    }
}


namespace ariles
{
    namespace compare
    {
        template<   class t_Visitor,
                    class t_Left,
                    class t_Right>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Visitor & visitor,
                    const t_Left & left,
                    const t_Right & right,
                    const std::string & name,
                    const typename t_Visitor::CompareParameters & param)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(left);
            ARILES_TRACE_TYPE(right);

            try
            {
                if (false == apply(visitor, left, right, param))
                {
                    ARILES_THROW("");
                }
            }
            catch (const std::exception & e)
            {
                ARILES_THROW("entry: " + name + " // " + std::string(e.what()));
            }
        }


        template<   class t_Visitor,
                    class t_Left,
                    class t_Right>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    t_Visitor & /*visitor*/,
                    const t_Left & left,
                    const t_Right & right,
                    const typename t_Visitor::CompareParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Left) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            if (true == param.compare_number_of_entries_)
            {
                ariles::count::Visitor counter;
                left.ariles(counter);

                const std::size_t left_counter = counter.counter_;
                right.ariles(counter);

                if (left_counter != counter.counter_)
                {
                    ARILES_THROW("Comparison failed: different number of entries.");
                }
            }
            return (left.arilesCompare(right, param));
        }


        template <  class t_Visitor,
                    typename t_Enumeration>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Visitor & /*visitor*/,
                    const t_Enumeration & left,
                    const t_Enumeration & right,
                    const typename t_Visitor::CompareParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            return (left == right);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Visitor> \
                    inline bool ARILES_VISIBILITY_ATTRIBUTE apply( \
                            const t_Visitor &, \
                            const type & left, \
                            const type & right, \
                            const typename t_Visitor::CompareParameters &) \
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



        template<class t_Visitor>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Visitor & visitor,
                    const float & left,
                    const float & right,
                    const typename t_Visitor::CompareParameters & param)
        {
            return (visitor.compareFloats(left, right, param));
        }


        template<class t_Visitor>
            bool ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Visitor & visitor,
                    const double & left,
                    const double & right,
                    const typename t_Visitor::CompareParameters & param)
        {
            return (visitor.compareFloats(left, right, param));
        }
    }
}


namespace ariles
{
    namespace defaults
    {
        template<   class t_Visitor,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::DefaultsParameters & param)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            apply(visitor, entry, param);
        }


        template<class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const typename t_Visitor::DefaultsParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry.arilesApply(visitor, param);
        }


        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Visitor & /*visitor*/,
                    t_Enumeration & entry,
                    const typename t_Visitor::DefaultsParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry = static_cast<t_Enumeration>(0);
        }


        #define ARILES_BASIC_TYPE(type) \
            template<class t_Visitor> \
                void ARILES_VISIBILITY_ATTRIBUTE apply( \
                        const t_Visitor &, \
                        type & entry, \
                        const typename t_Visitor::DefaultsParameters & param) \
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
        template<   class t_Visitor,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::FinalizeParameters & param)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            apply(visitor, entry, param);
        }


        template<class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                        const t_Visitor & visitor,
                        t_Entry & entry,
                        const typename t_Visitor::FinalizeParameters & param,
                        ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry.arilesApply(visitor, param);
            entry.finalize(); /// @todo DEPRECATED
        }


        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply(
                    const t_Visitor & /*visitor*/,
                    t_Enumeration & /*entry*/,
                    const typename t_Visitor::FinalizeParameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
        }


        #define ARILES_BASIC_TYPE(type) \
            template<class t_Visitor> \
                void ARILES_VISIBILITY_ATTRIBUTE apply( \
                        const t_Visitor &, \
                        const type &, \
                        const typename t_Visitor::FinalizeParameters &) \
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
        template<   class t_Visitor,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Visitor & visitor,
                    const t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::CountParameters & /*param*/,
                    ARILES_IS_CONFIGURABLE_DISABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_UNUSED_ARG(entry);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            ++visitor.counter_;
        }


        template<   class t_Visitor,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE arilesEntryApply(
                    t_Visitor & visitor,
                    const t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::CountParameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER_TYPE(t_Entry) * = NULL)
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            if (true == visitor.descend_)
            {
                visitor.descend_ = false;
                entry.arilesApply(visitor, param);
            }
            else
            {
                ++visitor.counter_;
            }
        }
    }
}
