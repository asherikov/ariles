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
    class Any : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "Any"
        #include ARILES_INITIALIZE

        protected:
            bool isConsitent() const
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
            std::string id_;
            t_Pointer<t_Base> value_;


        public:
            Any()
            {
                setDefaults();
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
                    if (config_section_id == value_->getConfigSectionID())
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
                    if (config_section_id == value_->getConfigSectionID())
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

            void writeConfigEntries(ariles::WriterBase & writer,
                                    const ariles::ConfigurableFlags & param) const
            {
                ARILES_ASSERT(
                        true == isConsitent(),
                        "Could not write config: entry is in an inconsistent (partially initialized) state.");

                ARILES_WRITE_ENTRY_(id);
                if (true == isInitialized())
                {
                    ARILES_WRITE_NAMED_ENTRY(*value_, "value");
                }
            }


            void readConfigEntries( ariles::ReaderBase & reader,
                                    const ariles::ConfigurableFlags & parameters)
            {
                ariles::ConfigurableFlags param = parameters;
                param.unset(ConfigurableFlags::ALLOW_MISSING_ENTRIES);

                ARILES_READ_ENTRY_(id);
                if ("" == id_)
                {
                    ARILES_ASSERT(
                            true == parameters.isSet(ConfigurableFlags::ALLOW_MISSING_ENTRIES),
                            "Id is empty, value cannot be read.");
                }
                else
                {
                    build(id_);
                    ARILES_READ_NAMED_ENTRY(*value_, "value");
                }
            }


            void arilesFinalize()
            {
                if (true == isInitialized())
                {
                    value_->finalize();
                }
            }

            void setDefaults()
            {
                id_ = "";
                value_.reset();
            }


            std::size_t getNumberOfEntries() const
            {
                return(2);
            }


            template<class t_Other>
            bool arilesCompare(const t_Other &other, const ariles::ComparisonParameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                if (true == param.compare_number_of_entries_)
                {
                    if (getNumberOfEntries() != other.getNumberOfEntries())
                    {
                        return (false);
                    }
                }

                if (this->id_ != other.id_)
                {
                    return (false);
                }

                if (NULL != value_.get() && NULL != other.value_.get())
                {
                    return (ariles::compare(*value_, *(other.value_), param));
                }
                else
                {
                    return (false);
                }
            }
    };
}


namespace ariles
{
    template <class t_Pointer>
    class NonNullPointer : public ariles::ConfigurableBase
    {
        #define ARILES_SECTION_ID "NonNullPointer"
        #include ARILES_INITIALIZE

        public:
            typedef t_Pointer BasePointer;
            typedef PointerHandler<t_Pointer> Handler;


        public:
            t_Pointer value_;


        public:
            NonNullPointer()
            {
                setDefaults();
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


            void writeConfigEntries(ariles::WriterBase &writer, const ariles::ConfigurableFlags &parameters) const
            {
                ARILES_ASSERT(false == isNull(), "Could not write config: entry is not initialized");
                value_->writeConfigEntries(writer, parameters);
            }


            void readConfigEntries(ariles::ReaderBase &reader, const ariles::ConfigurableFlags &parameters)
            {
                Handler::allocate(value_);
                value_->readConfigEntries(reader, parameters);
            }


            void arilesFinalize()
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
            }


            void setDefaults()
            {
                Handler::allocate(value_);
                value_->setDefaults();
            }


            std::size_t getNumberOfEntries() const
            {
                ARILES_ASSERT(false == isNull(), "Not initialized");
                return (value_->getNumberOfEntries());
            }


            bool isNull() const
            {
                return (Handler::isNull(value_));
            }


            template<class t_Other>
            bool arilesCompare(const t_Other &other, const ariles::ComparisonParameters &param) const
            {
                ARILES_TRACE_FUNCTION;
                return (value_->arilesCompare(*other.value_, param));
            }
    };
}
