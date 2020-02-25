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
    class Any : public ariles::DefaultBase
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
                ariles::apply<ariles::Defaults>(*this);
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

            void arilesVisit(   ariles::Write & writer,
                                const ariles::Write::Parameters & param) const
            {
                ARILES_ASSERT(
                        true == isConsistent(),
                        "Could not write config: entry is in an inconsistent (partially initialized) state.");

                writer(id_, "id", param);
                if (true == isInitialized())
                {
                    writer(*value_, "value", param);
                }
            }


            void arilesVisit(   ariles::Read & visitor,
                                const ariles::Read::Parameters & parameters)
            {
                ARILES_TRACE_FUNCTION;

                ariles::Read::Parameters param = parameters;
                param.unset(ariles::Read::Parameters::ALLOW_MISSING_ENTRIES);

                visitor(id_, "id", param);
                if ("" == id_)
                {
                    ARILES_ASSERT(
                            true == parameters.isSet(ariles::Read::Parameters::ALLOW_MISSING_ENTRIES),
                            "Id is empty, value cannot be read.");
                }
                else
                {
                    build(id_);
                    visitor(*value_, "value", param);
                }
            }


            void arilesVisit(   const ariles::PostProcess & visitor,
                                const ariles::PostProcess::Parameters & param)
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
    class NonNullPointer : public ariles::DefaultBase
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
                ariles::apply<ariles::Defaults>(*this);
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


            void arilesVisit(ariles::Write &writer, const ariles::Write::Parameters &parameters) const
            {
                ARILES_ASSERT(false == isNull(), "Could not write config: entry is not initialized");
                value_->arilesVisit(writer, parameters);
            }


            void arilesVisit(ariles::Read &reader, const ariles::Read::Parameters &parameters)
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                value_->arilesVisit(reader, parameters);
            }


            void arilesVisit(   const ariles::PostProcess & visitor,
                                const ariles::PostProcess::Parameters & param)
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                value_->arilesVisit(visitor, param);
            }

            void arilesVisit(   const ariles::Defaults & visitor,
                                const ariles::Defaults::Parameters & param)
            {
                Handler::allocate(value_);
                value_->arilesVisit(visitor, param);
            }


            void arilesVisit(   ariles::Count & visitor,
                                const ariles::Count::Parameters & param) const
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                value_->arilesVisit(visitor, param);
            }


            bool isNull() const
            {
                return (Handler::isNull(value_));
            }


            template<class t_Other>
            void arilesVisit(   ariles::Compare &visitor,
                                const t_Other &other,
                                const ariles::Compare::Parameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                value_->arilesVisit(visitor, *other.value_, param);
            }
    };
}
