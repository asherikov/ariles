/**
    @file
    @author  Alexander Sherikov

    @copyright 2019 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    template <template <class> class t_Pointer, class t_Base, class t_Instantiator>
    class ARILES2_VISIBILITY_ATTRIBUTE Any : public ariles2::DefaultBase
    {
#define ARILES2_ENTRIES(v)                                                                                             \
    ARILES2_TYPED_ENTRY_(v, id, std::string)                                                                           \
    ARILES2_TYPED_ENTRY_(v, value, t_Pointer<t_Base>)
#include ARILES2_INITIALIZE

    protected:
        bool isConsistent() const
        {
            if (("" != id_) && (NULL != value_.get()))
            {
                return (true);
            }

            if (("" == id_) && (NULL == value_.get()))
            {
                return (true);
            }

            return (false);
        }


    public:
        Any()
        {
            ariles2::apply<ariles2::Defaults>(*this);
        }


        explicit Any(const std::string &id)
        {
            build(id);
        }


        void build(const std::string &id)
        {
            id_ = id;
            value_ = t_Instantiator::instantiate(id_);
            ARILES2_ASSERT(NULL != value_.get(), "Could not instantiate class.");
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
            ARILES2_ASSERT(true == isInitialized(), "Not initialized");
            return (value_.get());
        }


        const t_Base *operator->() const
        {
            ARILES2_ASSERT(true == isInitialized(), "Not initialized");
            return (value_.get());
        }


        t_Base &operator*()
        {
            ARILES2_ASSERT(true == isInitialized(), "Not initialized");
            return (*value_);
        }


        const t_Base &operator*() const
        {
            ARILES2_ASSERT(true == isInitialized(), "Not initialized");
            return (*value_);
        }


        // Ariles methods

        void arilesVisit(ariles2::Write &visitor, const ariles2::Write::Parameters &param) const
        {
            ARILES2_ASSERT(
                    true == isConsistent(),
                    "Could not write config: entry is in an inconsistent (partially initialized) state.");

            visitor(id_, "id", param);
            if (true == isInitialized())
            {
                visitor(*value_, "value", param);
            }
        }


        void arilesVisit(ariles2::Read &visitor, const ariles2::Read::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;

            ariles2::Read::Parameters param = parameters;
            param.unset(ariles2::Read::Parameters::ALLOW_MISSING_ENTRIES);

            visitor(id_, "id", param);
            if ("" == id_)
            {
                ARILES2_ASSERT(
                        true == parameters.isSet(ariles2::Read::Parameters::ALLOW_MISSING_ENTRIES),
                        "Id is empty, value cannot be read.");
            }
            else
            {
                build(id_);
                visitor(*value_, "value", param);
            }
        }


        void arilesVisit(const ariles2::PostProcess &visitor, const ariles2::PostProcess::Parameters &param)
        {
            if (true == isInitialized())
            {
                value_->arilesVisit(visitor, param);
            }
        }


        void arilesVisit(const ariles2::PreProcess &visitor, const ariles2::PreProcess::Parameters &param)
        {
            if (true == isInitialized())
            {
                value_->arilesVisit(visitor, param);
            }
        }
    };
}  // namespace ariles2


namespace ariles2
{
    template <class t_Pointer>
    class ARILES2_VISIBILITY_ATTRIBUTE NonNullPointer : public ariles2::DefaultBase
    {
#include ARILES2_INITIALIZE

    public:
        typedef t_Pointer BasePointer;
        typedef PointerHandler<t_Pointer> Handler;


    public:
        t_Pointer value_;


    public:
        NonNullPointer()
        {
            ariles2::apply<ariles2::Defaults>(*this);
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
            ARILES2_ASSERT(false == isNull(), "Not initialized");
            return (value_.get());
        }


        typename Handler::Value &operator*() const
        {
            ARILES2_ASSERT(false == isNull(), "Not initialized");
            return (*value_);
        }


        void arilesVisit(ariles2::Write &writer, const ariles2::Write::Parameters &parameters) const
        {
            ARILES2_ASSERT(false == isNull(), "Could not write config: entry is not initialized");
            value_->arilesVisit(writer, parameters);
        }


        void arilesVisit(ariles2::Read &reader, const ariles2::Read::Parameters &parameters)
        {
            ARILES2_ASSERT(false == isNull(), "Not initialized");
            value_->arilesVisit(reader, parameters);
        }


        void arilesVisit(const ariles2::PostProcess &visitor, const ariles2::PostProcess::Parameters &param)
        {
            ARILES2_ASSERT(false == isNull(), "Not initialized");
            value_->arilesVisit(visitor, param);
        }

        void arilesVisit(const ariles2::PreProcess &visitor, const ariles2::PreProcess::Parameters &param)
        {
            if (false == isNull())
            {
                value_->arilesVisit(visitor, param);
            }
        }

        void arilesVisit(const ariles2::Defaults &visitor, const ariles2::Defaults::Parameters &param)
        {
            Handler::allocate(value_);
            value_->arilesVisit(visitor, param);
        }


        std::size_t arilesVisit(const ariles2::Count &visitor, const ariles2::Count::Parameters &param) const
        {
            ARILES2_ASSERT(false == isNull(), "Not initialized");
            return (value_->arilesVisit(visitor, param));
        }


        bool isNull() const
        {
            return (Handler::isNull(value_));
        }


        template <class t_Other>
        void arilesVisit(ariles2::Compare &visitor, const t_Other &other, const ariles2::Compare::Parameters &param)
                const
        {
            ARILES2_TRACE_FUNCTION;
            value_->arilesVisit(visitor, *other.value_, param);
        }
    };
}  // namespace ariles2
