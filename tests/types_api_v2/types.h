/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    template<template<class> class t_Pointer, class t_Base, class t_Instantiator>
    class Any : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "Any"
        #define ARILES_ENTRIES \
            ARILES_TYPED_ENTRY_(id, std::string) \
            ARILES_TYPED_ENTRY_(value, t_Pointer<t_Base>)
        #include ARILES_INITIALIZE

        protected:
            bool isConsistent() const
            {
                if (("" != id_) && (NULL != value_.get()))
                {
                    return(true);
                }

                if (("" == id_) && (NULL == value_.get()))
                {
                    return(true);
                }

                return (false);
            }


        public:
            Any()
            {
                arilesVirtualVisit<ariles::defaults::Visitor>();
            }


            Any(const std::string & id)
            {
                build(id);
            }


            void build(const std::string & id)
            {
                id_ = id;
                value_ = t_Instantiator::instantiate(id_);
                ARILES_ASSERT(NULL != value_.get(), "Could not instantiate class.");
            }


            bool isInitialized() const
            {
                return ("" != id_ && NULL != value_.get());
            }


            /// @{
            /**
             * @brief Cast methods are potentially dangerous, no id checks are
             * performed. If value is not initialized the returned pointer may
             * be NULL.
             */
            template <class t_Derived>
            t_Derived *cast()
            {
                return (dynamic_cast<t_Derived *>(value_.get()));
            }


            template <class t_Derived>
            const t_Derived *cast() const
            {
                return (dynamic_cast<const t_Derived *>(value_.get()));
            }
            /// @}


            /// @{
            /**
             * @brief These casts succeed if the Ariles config section id
             * matches the given string.
             */
            template <class t_Derived>
            t_Derived *cast(const std::string &config_section_id)
            {
                if (true == isInitialized())
                {
                    if (config_section_id == value_->arilesDefaultID())
                    {
                        return (dynamic_cast<t_Derived *>(value_.get()));
                    }
                }
                return (NULL);
            }


            template <class t_Derived>
            const t_Derived *cast(const std::string &config_section_id) const
            {
                if (true == isInitialized())
                {
                    if (config_section_id == value_->arilesDefaultID())
                    {
                        return (dynamic_cast<t_Derived *>(value_.get()));
                    }
                }
                return (NULL);
            }
            /// @}


            t_Base *operator->()
            {
                ARILES_ASSERT(true == isInitialized(), "Not initialized");
                return(value_.get());
            }


            const t_Base *operator->() const
            {
                ARILES_ASSERT(true == isInitialized(), "Not initialized");
                return(value_.get());
            }


            t_Base &operator*()
            {
                ARILES_ASSERT(true == isInitialized(), "Not initialized");
                return(*value_);
            }


            const t_Base &operator*() const
            {
                ARILES_ASSERT(true == isInitialized(), "Not initialized");
                return(*value_);
            }


        // Ariles methods

            void arilesVisit(   ariles::write::Visitor & writer,
                                const ariles::write::Visitor::Parameters & param) const
            {
                ARILES_ASSERT(
                        true == isConsistent(),
                        "Could not write config: entry is in an inconsistent (partially initialized) state.");

                arilesEntryApply(writer, id_, "id", param);
                if (true == isInitialized())
                {
                    arilesEntryApply(writer, *value_, "value", param);
                }
            }


            void arilesVisit(   ariles::read::Visitor & visitor,
                                const ariles::read::Visitor::Parameters & parameters)
            {
                ARILES_TRACE_FUNCTION;

                ariles::read::Visitor::Parameters param = parameters;
                param.unset(ariles::read::Visitor::Parameters::ALLOW_MISSING_ENTRIES);

                arilesEntryApply(visitor, id_, "id", param);
                if ("" == id_)
                {
                    ARILES_ASSERT(
                            true == parameters.isSet(ariles::read::Visitor::Parameters::ALLOW_MISSING_ENTRIES),
                            "Id is empty, value cannot be read.");
                }
                else
                {
                    build(id_);
                    arilesEntryApply(visitor, *value_, "value", param);
                }
            }


            void arilesVisit(   const ariles::finalize::Visitor & visitor,
                                const ariles::finalize::Visitor::Parameters & param)
            {
                if (true == isInitialized())
                {
                    value_->arilesVisit(visitor, param);
                }
            }
    };
}


namespace ariles
{
    template <class t_Pointer>
    class NonNullPointer : public ariles::Base
    {
        #define ARILES_DEFAULT_ID "NonNullPointer"
        #include ARILES_INITIALIZE

        public:
            typedef t_Pointer BasePointer;
            typedef PointerHandler<t_Pointer> Handler;


        public:
            t_Pointer value_;


        public:
            NonNullPointer()
            {
                arilesVirtualVisit<ariles::defaults::Visitor>();
            }


            NonNullPointer(const t_Pointer &value)
            {
                value_ = value;
            }


            operator BasePointer &()
            {
                return (value_);
            }

            operator const BasePointer &() const
            {
                return (value_);
            }


            NonNullPointer(const typename Handler::Value &value)
            {
                Handler::allocate(value_);
                *value_ = value;
            }


            virtual ~NonNullPointer()
            {
            }


            typename Handler::Value *operator->() const
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                return (value_.get());
            }


            typename Handler::Value &operator*() const
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                return (*value_);
            }


            void arilesVisit(ariles::write::Visitor &writer, const ariles::write::Visitor::Parameters &parameters) const
            {
                ARILES_ASSERT(false == isNull(), "Could not write config: entry is not initialized");
                value_->arilesVisit(writer, parameters);
            }


            void arilesVisit(ariles::read::Visitor &reader, const ariles::read::Visitor::Parameters &parameters)
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                value_->arilesVisit(reader, parameters);
            }


            void arilesVisit(   const ariles::finalize::Visitor & visitor,
                                const ariles::finalize::Visitor::Parameters & param)
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                value_->arilesVisit(visitor, param);
            }

            void arilesVisit(   const ariles::defaults::Visitor & visitor,
                                const ariles::defaults::Visitor::Parameters & param)
            {
                Handler::allocate(value_);
                value_->arilesVisit(visitor, param);
            }


            void arilesVisit(   ariles::count::Visitor & visitor,
                                const ariles::count::Visitor::Parameters & param) const
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                value_->arilesVisit(visitor, param);
            }


            bool isNull() const
            {
                return (Handler::isNull(value_));
            }


            template<class t_Other>
            void arilesVisit(   ariles::compare::Visitor &visitor,
                                const t_Other &other,
                                const ariles::compare::Visitor::Parameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                value_->arilesVisit(visitor, *other.value_, param);
            }
    };
}
