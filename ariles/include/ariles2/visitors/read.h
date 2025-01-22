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
        using Parameters = serialization::Parameters;


        class Visitor : public serialization::Base<Visitor, Parameters>
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
                        CPPUT_ASSERT(size == min, "Actual number of entries is not the same as expected.");
                        return;
                    case SIZE_LIMIT_RANGE:
                        CPPUT_ASSERT(min <= size, "Actual number of entries is lower than expected.");
                        CPPUT_ASSERT(max >= size, "Actual number of entries is larger than expected.");
                        return;
                    case SIZE_LIMIT_MIN:
                        CPPUT_ASSERT(min <= size, "Actual number of entries is lower than expected.");
                        return;
                    default:
                        CPPUT_THROW("Internal logic error.");
                }
            }


            virtual bool startRoot(const std::string &name)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(name);

                if (not name.empty())
                {
                    return (startMapEntry(name));
                }
                return (true);
            }

            virtual void endRoot(const std::string &name)
            {
                CPPUT_TRACE_FUNCTION;
                if (not name.empty())
                {
                    endMapEntry();
                }
            }


            virtual bool startRoot(const std::vector<std::string> &subtree)
            {
                CPPUT_TRACE_FUNCTION;

                bool result = false;
                if (0 == subtree.size())
                {
                    result = this->startRoot("");
                }
                else
                {
                    result = this->startRoot(subtree[0]);
                    for (std::size_t i = 1; i < subtree.size() and result; ++i)
                    {
                        this->startMap(SIZE_LIMIT_MIN, 1);
                        result &= (this->startMapEntry(subtree[i]));
                    }
                }

                return (result);
            }

            virtual void endRoot(const std::vector<std::string> &subtree)
            {
                CPPUT_TRACE_FUNCTION;
                for (std::size_t i = 1; i < subtree.size(); ++i)
                {
                    this->endMapEntry();
                    this->endMap();
                }
                if (0 == subtree.size())
                {
                    endRoot("");
                }
                else
                {
                    endRoot(subtree[0]);
                }
            }


            const std::string &convertSubtreeToString(const std::string &subtree) const
            {
                return (subtree);
            }

            std::string convertSubtreeToString(const std::vector<std::string> &subtree) const
            {
                std::string result;
                for (std::size_t i = 0; i < subtree.size(); ++i)
                {
                    if (i > 0)
                    {
                        result += "/";
                    }
                    result += subtree[i];
                }
                return (result);
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
                CPPUT_TRACE_FUNCTION;
                CPPUT_UNUSED_ARG(child_name)
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
                CPPUT_THROW("startIteratedMapElement() is not supported.");
            }
            virtual void endIteratedMapElement()
            {
            }
            virtual void endIteratedMap()
            {
            }


            bool startPointer(const Parameters &param)
            {
                CPPUT_TRACE_FUNCTION;

                bool is_null = true;

                this->startMap(SIZE_LIMIT_RANGE, 1, 2);

                if (this->startMapEntry("is_null"))
                {
                    this->readElement(is_null);
                    this->endMapEntry();
                }
                else
                {
                    CPPUT_PERSISTENT_ASSERT(
                            param.allow_missing_entries_, "Pointer entry does not include 'is_null' subentry.");
                }

                if (not is_null)
                {
                    CPPUT_ASSERT(this->startMapEntry("value"), "Missing value in a pointer entry.");
                }

                return (is_null);
            }
            void endPointer(const bool is_null)
            {
                if (not is_null)
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
                if (param.flat_matrices_)
                {
                    if (dynamic or param.explicit_matrix_size_)
                    {
                        this->startMap(SIZE_LIMIT_EQUAL, 3);

                        CPPUT_ASSERT(this->startMapEntry("cols"), "Missing 'cols' in a matrix entry.");
                        this->readElement(cols);
                        this->endMapEntry();

                        CPPUT_ASSERT(this->startMapEntry("rows"), "Missing 'rows' in a matrix entry.");
                        this->readElement(rows);
                        this->endMapEntry();

                        CPPUT_ASSERT(this->startMapEntry("data"), "Missing 'data' in a matrix entry.");

                        // cppcheck-suppress unreadVariable
                        const std::size_t vec_len = this->startVector();
                        CPPUT_ASSERT(cols * rows == vec_len, "Inconsistent matrix size.");
                    }
                    else
                    {
                        this->startVector();
                    }
                }
                else
                {
                    rows = this->startArray();
                    if (rows > 0)
                    {
                        this->startArrayElement();
                        cols = this->startVector();
                    }
                }
            }
            virtual void startMatrixRow(const std::size_t row_index, const std::size_t cols, const Parameters &param)
            {
                if (not param.flat_matrices_ and 0 != row_index)
                {
                    this->startArrayElement();
                    // cppcheck-suppress unreadVariable
                    const std::size_t vec_len = this->startVector();
                    CPPUT_ASSERT(cols == vec_len, "Inconsistent matrix row length.");
                }
            }
            virtual void startMatrixElement()
            {
                this->startVectorElement();
            }
            virtual void endMatrixElement()
            {
                this->endVectorElement();
            }
            virtual void endMatrixRow(const Parameters &param)
            {
                if (not param.flat_matrices_)
                {
                    this->endVector();
                    this->endArrayElement();
                }
            }
            virtual void endMatrix(const bool dynamic, const Parameters &param)
            {
                if (param.flat_matrices_)
                {
                    this->endVector();

                    if (dynamic)
                    {
                        this->endMapEntry();
                        this->endMap();
                    }
                }
                else
                {
                    this->endArray();
                }
            }


            template <class t_Scalar>
            void readElement(std::complex<t_Scalar> &entry)
            {
                CPPUT_PERSISTENT_ASSERT(2 == this->startArray(), "Wrong number of elements in a complex number");
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


            template <class t_Entry, class t_Subtree>
            void visit(t_Entry &entry, const t_Subtree &subtree, const Parameters &param)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_TYPE(entry);


                if (this->startRoot(subtree))
                {
                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        CPPUT_THROW("Failed to parse entry <", convertSubtreeToString(subtree), "> ||  ", e.what());
                    }

                    this->endRoot(subtree);
                }
                else
                {
                    CPPUT_PERSISTENT_ASSERT(
                            param.allow_missing_entries_,
                            "Configuration file does not contain entry '",
                            convertSubtreeToString(subtree),
                            "'.");
                }
            }


            template <class t_Entry>
            bool visitMapEntry(
                    t_Entry &entry,
                    const std::string &name,
                    const Parameters &param,
                    const bool override_missing_entries_locally = false)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_VALUE(name);
                CPPUT_TRACE_TYPE(entry);

                if (this->startMapEntry(name))
                {
                    try
                    {
                        apply_read(*this, entry, param);
                    }
                    catch (const std::exception &e)
                    {
                        CPPUT_THROW("Failed to parse entry <", name, "> ||  ", e.what());
                    }

                    this->endMapEntry();
                    return (true);
                }
                else
                {
                    CPPUT_PERSISTENT_ASSERT(
                            not override_missing_entries_locally and param.allow_missing_entries_,
                            "Configuration file does not contain entry '",
                            name,
                            "'.");
                    return (false);
                }
            }

            template <typename t_Element>
            void visitArrayElement(t_Element &element, const Parameters &param)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_TYPE(element);

                this->startArrayElement();
                apply_read(*this, element, param);
                this->endArrayElement();
            }

            template <typename t_Element>
            void visitVectorElement(t_Element &element, const Parameters & /*param*/)
            {
                CPPUT_TRACE_FUNCTION;
                CPPUT_TRACE_TYPE(element);

                this->startVectorElement();
                this->readElement(element);
                this->endVectorElement();
            }

            template <typename t_Element>
            void visitMatrixElement(t_Element &element, const Parameters & /*param*/)
            {
                CPPUT_TRACE_FUNCTION;

                this->startMatrixElement();
                this->readElement(element);
                this->endMatrixElement();
            }
        };


        class FileVisitorImplementation
        {
        protected:
            std::ifstream config_ifs_;

            std::istream *input_stream_;

        protected:
            FileVisitorImplementation() = default;

            explicit FileVisitorImplementation(const std::string &file_name)
            {
                openFile(file_name);
                input_stream_ = &config_ifs_;
            }

            explicit FileVisitorImplementation(std::istream &input_stream)
            {
                input_stream_ = &input_stream;
            }


            /**
             * @brief open configuration file
             *
             * @param[in] file_name
             */
            void openFile(const std::string &file_name)
            {
                config_ifs_.open(file_name.c_str());
                if (!config_ifs_.good())
                {
                    std::string file_name_default = file_name;
                    config_ifs_.open(file_name_default.c_str());
                }
                CPPUT_PERSISTENT_ASSERT(config_ifs_.good(), "Could not open configuration file: ", file_name.c_str());
            }
        };


        using Base = entry::Base<read::Visitor>;


#define ARILES2_NAMED_ENTRY_read(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_read(v, entry)
#define ARILES2_VISIT_read                                                                                             \
    template <class t_Visitor> /* cppcheck-suppress duplInheritedMember */                                             \
    void arilesVisit(                                                                                                  \
            t_Visitor &visitor,                                                                                        \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::read::Visitor, t_Visitor))                                                \
    {                                                                                                                  \
        CPPUT_TRACE_FUNCTION;                                                                                          \
        CPPUT_UNUSED_ARG(visitor);                                                                                     \
        CPPUT_UNUSED_ARG(parameters);                                                                                  \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(read)                                                                                          \
    }

#define ARILES2_METHODS_read ARILES2_METHODS(read, ARILES2_EMPTY_MACRO, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_read ARILES2_BASE_METHODS(read)
    }  // namespace read


    /// @ingroup read
    using Read = read::Visitor;
}  // namespace ariles2
