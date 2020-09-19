/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <vector>

#include "serialization.h"
#include "count.h"

/**
@defgroup read Read
@ingroup serialization

@brief Base for deserialization.
*/

namespace ariles2
{
    /// @ingroup read
    namespace read
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public serialization::Parameters
        {
        public:
            bool allow_missing_entries_;


        public:
            Parameters(const bool override_parameters = true) : serialization::Parameters(override_parameters)
            {
                allow_missing_entries_ = false;
            }
        };



        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public serialization::Base<Visitor, Parameters>
        {
        public:
            enum SizeLimitEnforcementType
            {
                SIZE_LIMIT_UNDEFINED = 0,
                SIZE_LIMIT_NONE = 1,
                SIZE_LIMIT_EQUAL = 2,
                SIZE_LIMIT_RANGE = 3,
                SIZE_LIMIT_MIN = 4
            };


        protected:
            Visitor(){};
            ~Visitor(){};


            void checkSize(
                    const SizeLimitEnforcementType limit_type,
                    const std::size_t size = 0,
                    const std::size_t min = 0,
                    const std::size_t max = 0) const
            {
                switch (limit_type)
                {
                    case SIZE_LIMIT_NONE:
                        return;
                    case SIZE_LIMIT_EQUAL:
                        ARILES2_ASSERT(size == min, "Actual number of entries is not the same as expected.");
                        return;
                    case SIZE_LIMIT_RANGE:
                        ARILES2_ASSERT(min <= size, "Actual number of entries is lower than expected.");
                        ARILES2_ASSERT(max >= size, "Actual number of entries is larger than expected.");
                        return;
                    case SIZE_LIMIT_MIN:
                        ARILES2_ASSERT(min <= size, "Actual number of entries is lower than expected.");
                        return;
                    default:
                        ARILES2_THROW("Internal logic error.");
                        return;
                }
            }


            /**
             * @brief open configuration file
             *
             * @param[out] config_ifs
             * @param[in] file_name
             */
            static void openFile(std::ifstream &config_ifs, const std::string &file_name)
            {
                config_ifs.open(file_name.c_str());
                if (!config_ifs.good())
                {
                    std::string file_name_default = file_name;
                    config_ifs.open(file_name_default.c_str());
                }
                ARILES2_PERSISTENT_ASSERT(
                        true == config_ifs.good(),
                        std::string("Could not open configuration file: ") + file_name.c_str());
            }


            virtual bool startRoot(const std::string &name)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);

                if (false == name.empty())
                {
                    return (startMapEntry(name));
                }
                return (true);
            }

            virtual void endRoot(const std::string &name)
            {
                ARILES2_TRACE_FUNCTION;
                if (false == name.empty())
                {
                    endMapEntry();
                }
            }


        public:
            template <class t_Entry>
            void startMap(t_Entry &entry, const Parameters &parameters)
            {
                startMap(
                        parameters.allow_missing_entries_ ? SIZE_LIMIT_NONE : SIZE_LIMIT_MIN,
                        ariles2::apply<ariles2::Count>(entry));
            }

            virtual void startMap(
                    const SizeLimitEnforcementType /*limit_type*/ = SIZE_LIMIT_NONE,
                    const std::size_t /*min*/ = 0,
                    const std::size_t /*max*/ = 0)
            {
            }

            /**
             * @brief startMapEntry to the entry with the given name
             *
             * @param[in] child_name child node name
             *
             * @return true if successful.
             */
            virtual bool startMapEntry(const std::string &child_name)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_UNUSED_ARG(child_name)
                return (true);
            }

            /**
             * @brief endMapEntry from the current entry to its parent.
             */
            virtual void endMapEntry() = 0;

            virtual void endMap()
            {
            }


            virtual bool startIteratedMap(
                    const SizeLimitEnforcementType /*limit_type*/ = SIZE_LIMIT_NONE,
                    const std::size_t /*min*/ = 0,
                    const std::size_t /*max*/ = 0)
            {
                return (false);
            }
            virtual bool startIteratedMapElement(std::string & /*entry_name*/)
            {
                ARILES2_THROW("startIteratedMapElement() is not supported.");
                return (false);
            }
            virtual void endIteratedMapElement()
            {
            }
            virtual void endIteratedMap()
            {
            }


            bool startPointer(const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;

                bool is_null = true;

                this->startMap(SIZE_LIMIT_RANGE, 1, 2);

                if (this->startMapEntry("is_null"))
                {
                    this->readElement(is_null);
                    this->endMapEntry();
                }
                else
                {
                    ARILES2_PERSISTENT_ASSERT(
                            true == param.allow_missing_entries_,
                            std::string("Pointer entry does not include 'is_null' subentry."));
                }

                if (false == is_null)
                {
                    ARILES2_ASSERT(true == this->startMapEntry("value"), "Missing value in a pointer entry.");
                }

                return (is_null);
            }
            void endPointer(const bool is_null)
            {
                if (false == is_null)
                {
                    this->endMapEntry();
                }
                this->endMap();
            }


            virtual std::size_t startArray() = 0;
            virtual void startArrayElement(){};
            virtual void endArrayElement() = 0;
            virtual void endArray() = 0;

            virtual std::size_t startVector()
            {
                return (startArray());
            }
            virtual void startVectorElement()
            {
                startArrayElement();
            }
            virtual void endVectorElement()
            {
                endArrayElement();
            }
            virtual void endVector()
            {
                endArray();
            }


            virtual void startMatrix(std::size_t &cols, std::size_t &rows, const bool dynamic, const Parameters &param)
            {
                if (true == dynamic or true == param.explicit_matrix_size_)
                {
                    this->startMap(SIZE_LIMIT_EQUAL, 3);

                    ARILES2_ASSERT(true == this->startMapEntry("cols"), "Missing 'cols' in a matrix entry.");
                    this->readElement(cols);
                    this->endMapEntry();

                    ARILES2_ASSERT(true == this->startMapEntry("rows"), "Missing 'rows' in a matrix entry.");
                    this->readElement(rows);
                    this->endMapEntry();

                    ARILES2_ASSERT(true == this->startMapEntry("data"), "Missing 'data' in a matrix entry.");
                }
            }
            virtual void endMatrix(const bool dynamic)
            {
                if (true == dynamic)
                {
                    this->endMapEntry();
                    this->endMap();
                }
            }


            template <class t_Scalar>
            void readElement(std::complex<t_Scalar> &entry)
            {
                ARILES2_PERSISTENT_ASSERT(2 == this->startArray(), "Wrong number of elements in a complex number");
                t_Scalar value;
                this->startArrayElement();
                this->readElement(value);
                entry.real(value);
                this->endArrayElement();
                this->startArrayElement();
                this->readElement(value);
                entry.imag(value);
                this->endArrayElement();
                this->endArray();
            }
            virtual void readElement(std::complex<float> &entry)
            {
                readElement<float>(entry);
            }
            virtual void readElement(std::complex<double> &entry)
            {
                readElement<double>(entry);
            }


#define ARILES2_BASIC_TYPE(type) virtual void readElement(type &entry) = 0;

            ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE


            template <class t_Entry>
            void visit(t_Entry &entry, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(entry);


                if (this->startRoot(name))
                {
                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        ARILES2_THROW(std::string("Failed to parse entry <") + name + "> ||  " + e.what());
                    }

                    this->endRoot(name);
                }
                else
                {
                    ARILES2_PERSISTENT_ASSERT(
                            true == param.allow_missing_entries_,
                            std::string("Configuration file does not contain entry '") + name + "'.");
                }
            }


            template <class t_Entry>
            bool visitMapEntry(
                    t_Entry &entry,
                    const std::string &name,
                    const Parameters &param,
                    const bool override_missing_entries_locally = false)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(entry);

                if (this->startMapEntry(name))
                {
                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        ARILES2_THROW(std::string("Failed to parse entry <") + name + "> ||  " + e.what());
                    }

                    this->endMapEntry();
                    return (true);
                }
                else
                {
                    ARILES2_PERSISTENT_ASSERT(
                            false == override_missing_entries_locally and true == param.allow_missing_entries_,
                            std::string("Configuration file does not contain entry '") + name + "'.");
                    return (false);
                }
            }

            template <typename t_Element>
            void visitArrayElement(t_Element &element, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_TYPE(element);

                this->startArrayElement();
                apply_read(*this, element, param);
                this->endArrayElement();
            }

            template <typename t_Element>
            void visitVectorElement(t_Element &element, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_TYPE(element);

                this->startVectorElement();
                apply_read(*this, element, param);
                this->endVectorElement();
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::Base<read::Visitor>
        {
        };


#define ARILES2_NAMED_ENTRY_read(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_read(v, entry)
#define ARILES2_VISIT_read                                                                                             \
    template <class t_Visitor>                                                                                         \
    void arilesVisit(                                                                                                  \
            t_Visitor &visitor,                                                                                        \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::read::Visitor, t_Visitor))                                                \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(read)                                                                                          \
    }

#define ARILES2_METHODS_read ARILES2_METHODS(read, ARILES2_EMPTY_MACRO, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_read ARILES2_BASE_METHODS(read)
    }  // namespace read


    /// @ingroup read
    typedef read::Visitor Read;
}  // namespace ariles2
