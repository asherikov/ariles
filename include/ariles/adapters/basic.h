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
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    t_Visitor & visitor,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_BASE_ENABLER(ariles::read::Visitor, t_Visitor))
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            if (visitor.descend(name))
            {
                try
                {
                    apply_read(visitor, entry, param);
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
                ARILES_PERSISTENT_ASSERT(   true == param.isSet(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES),
                                            std::string("Configuration file does not contain entry '") + name + "'.");
            }
        }



        template <class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor & visitor,
                    t_Entry & entry,
                    const typename t_Visitor::Parameters & parameters,
                    ARILES_IS_CONFIGURABLE_ENABLER(t_Entry))
        {
            ARILES_TRACE_FUNCTION;

            typename t_Visitor::Parameters param = parameters;
            if (false == param.isSet(t_Visitor::Parameters::PROPAGATE_ALLOW_MISSING_ENTRIES))
            {
                param.set(t_Visitor::Parameters::DEFAULT & t_Visitor::Parameters::ALLOW_MISSING_ENTRIES);
            }

            ariles::count::Visitor counter;
            ariles::apply(counter, entry);
            if (true == param.isSet(t_Visitor::Parameters::ALLOW_MISSING_ENTRIES))
            {
                visitor.template startMap<t_Visitor::SIZE_LIMIT_NONE>(counter.counter_);
            }
            else
            {
                visitor.template startMap<t_Visitor::SIZE_LIMIT_EQUAL>(counter.counter_);
            }
            entry.arilesVirtualVisit(visitor, param);
            visitor.endMap();

#if 1 == ARILES_API_VERSION
            entry.finalize(); /// @todo DEPRECATED
#endif
        }


        /**
         * @brief Read configuration entry (an enum)
         * This function is necessary since an explicit casting to integer is needed.
         */
        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                    t_Visitor & visitor,
                    t_Enumeration &entry,
                    const typename t_Visitor::Parameters & /*param*/,
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
                    void ARILES_VISIBILITY_ATTRIBUTE apply_read( \
                                    t_Visitor & visitor, \
                                    type &entry, \
                                    const typename t_Visitor::Parameters & param) \
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
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    t_Visitor & writer,
                    const t_Entry & entry,
                    const std::string & entry_name,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_BASE_ENABLER(ariles::write::Visitor, t_Visitor))
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(entry_name);
            ARILES_TRACE_TYPE(entry);

            writer.descend(entry_name);
            apply_write(writer, entry, param);
            writer.ascend();
        }



        template <class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const t_Entry & entry,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER(t_Entry))
        {
            ARILES_TRACE_FUNCTION;
            ariles::count::Visitor counter;
            ariles::apply(counter, entry);
            writer.startMap(counter.counter_);
            entry.arilesVirtualVisit(writer, param);
            writer.endMap();
        }



        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                    t_Visitor & writer,
                    const t_Enumeration entry,
                    const typename t_Visitor::Parameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            int tmp_value = entry;
            writer.writeElement(tmp_value);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Visitor> \
                    void ARILES_VISIBILITY_ATTRIBUTE apply_write( \
                            t_Visitor &  writer, \
                            const type & entry, \
                            const typename t_Visitor::Parameters &) \
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
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    t_Visitor & visitor,
                    const t_Left & left,
                    const t_Right & right,
                    const std::string & name,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_BASE_ENABLER(ariles::compare::Visitor, t_Visitor))
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(left);
            ARILES_TRACE_TYPE(right);

            try
            {
                apply_compare(visitor, left, right, param);
                if (false == visitor.equal_ and true == param.throw_on_error_)
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
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const t_Left & left,
                    const t_Right & right,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER(t_Left))
        {
            ARILES_TRACE_FUNCTION;
            if (true == param.compare_number_of_entries_)
            {
                ariles::count::Visitor counter;
                ariles::apply(counter, left);

                const std::size_t left_counter = counter.counter_;
                ariles::apply(counter, right);

                visitor.equal_ &= (left_counter == counter.counter_);
            }
            left.arilesVisit(visitor, right, param);
        }


        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const t_Enumeration & left,
                    const t_Enumeration & right,
                    const typename t_Visitor::Parameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            visitor.equal_ &= (left == right);
        }


        #define ARILES_BASIC_TYPE(type) \
                template <class t_Visitor> \
                    inline void ARILES_VISIBILITY_ATTRIBUTE apply_compare( \
                            t_Visitor & visitor, \
                            const type & left, \
                            const type & right, \
                            const typename t_Visitor::Parameters &) \
                { visitor.equal_ &= (left == right); }

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
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const float & left,
                    const float & right,
                    const typename t_Visitor::Parameters & param)
        {
            visitor.equal_ &= (visitor.compareFloats(left, right, param));
        }


        template<class t_Visitor>
            void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                    t_Visitor & visitor,
                    const double & left,
                    const double & right,
                    const typename t_Visitor::Parameters & param)
        {
            visitor.equal_ &= visitor.compareFloats(left, right, param);
        }
    }
}


namespace ariles
{
    namespace defaults
    {
        template<   class t_Visitor,
                    class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_BASE_ENABLER(ariles::defaults::Visitor, t_Visitor))
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            apply_defaults(visitor, entry, param);
        }


        template<class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER(t_Entry))
        {
            ARILES_TRACE_FUNCTION;
            entry.arilesVirtualVisit(visitor, param);
        }


        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                    const t_Visitor & /*visitor*/,
                    t_Enumeration & entry,
                    const typename t_Visitor::Parameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
            entry = static_cast<t_Enumeration>(0);
        }


        #define ARILES_BASIC_TYPE(type) \
            template<class t_Visitor> \
                void ARILES_VISIBILITY_ATTRIBUTE apply_defaults( \
                        const t_Visitor &, \
                        type & entry, \
                        const typename t_Visitor::Parameters & param) \
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
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_BASE_ENABLER(ariles::finalize::Visitor, t_Visitor))
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            apply_finalize(visitor, entry, param);
        }


        template<class t_Visitor, class t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE apply_finalize(
                    const t_Visitor & visitor,
                    t_Entry & entry,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER(t_Entry))
        {
            ARILES_TRACE_FUNCTION;
            entry.arilesVirtualVisit(visitor, param);
#if 1 == ARILES_API_VERSION
            entry.finalize(); /// @todo DEPRECATED
#endif
        }


        template <  class t_Visitor,
                    typename t_Enumeration>
            void ARILES_VISIBILITY_ATTRIBUTE apply_finalize(
                    const t_Visitor & /*visitor*/,
                    t_Enumeration & /*entry*/,
                    const typename t_Visitor::Parameters & /*param*/,
                    ARILES_IS_ENUM_ENABLER_TYPE(t_Enumeration) * = NULL)
        {
            ARILES_TRACE_FUNCTION;
        }


        #define ARILES_BASIC_TYPE(type) \
            template<class t_Visitor> \
                void ARILES_VISIBILITY_ATTRIBUTE apply_finalize( \
                        const t_Visitor &, \
                        const type &, \
                        const typename t_Visitor::Parameters &) \
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
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    t_Visitor & visitor,
                    const t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::Parameters & /*param*/,
                    ARILES_IS_CONFIGURABLE_DISABLER(t_Entry),
                    ARILES_IS_BASE_ENABLER(ariles::count::Visitor, t_Visitor))
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
            void ARILES_VISIBILITY_ATTRIBUTE applyToEntry(
                    t_Visitor & visitor,
                    const t_Entry & entry,
                    const std::string & name,
                    const typename t_Visitor::Parameters & param,
                    ARILES_IS_CONFIGURABLE_ENABLER(t_Entry),
                    ARILES_IS_BASE_ENABLER(ariles::count::Visitor, t_Visitor))
        {
            ARILES_UNUSED_ARG(name);
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            ARILES_TRACE_TYPE(entry);
            if (true == visitor.descend_)
            {
                visitor.descend_ = false;
                entry.arilesVirtualVisit(visitor, param);
            }
            else
            {
                ++visitor.counter_;
            }
        }
    }
}
