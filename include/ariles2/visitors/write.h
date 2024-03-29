/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "serialization.h"
#include "count.h"
#include "count_missing.h"

/**
@defgroup write Write
@ingroup serialization

@brief Base for serialization.
*/

namespace ariles2
{
    /// @ingroup write
    namespace write
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters : public serialization::Parameters
        {
        public:
            bool compact_arrays_;


        public:
            Parameters(const bool override_parameters = true) : serialization::Parameters(override_parameters)
            {
                compact_arrays_ = false;
            }
        };



        template <class t_Derived, class t_Parameters>
        class ARILES2_VISIBILITY_ATTRIBUTE VisitorBase : public serialization::Base<t_Derived, t_Parameters>
        {
        protected:
            VisitorBase(){};
            ~VisitorBase(){};


        public:
            /**
             * @brief open configuration file
             *
             * @param[out] config_ofs
             * @param[in] file_name
             */
            static void openFile(std::ofstream &config_ofs, const std::string &file_name)
            {
                config_ofs.open(file_name.c_str());

                ARILES2_PERSISTENT_ASSERT(
                        config_ofs.good(),
                        std::string("Could not open configuration file for writing: ") + file_name.c_str());
            }


        public:
            virtual void startRoot(const std::string &name, const t_Parameters & /*param*/)
            {
                ARILES2_TRACE_FUNCTION;
                if (not name.empty())
                {
                    startMapEntry(name);
                }
            }
            virtual void endRoot(const std::string &name)
            {
                ARILES2_TRACE_FUNCTION;
                if (not name.empty())
                {
                    endMapEntry();
                }
            }
            /**
             * @brief Flush the configuration to the output
             */
            virtual void flush() = 0;


            /**
             * @brief Starts a nested map in the configuration file
             *
             * @param[in] instance_id instance id
             * @param[in] num_entries number of child entries
             */
            virtual void startMap(const t_Parameters & /*param*/, const std::size_t /*num_entries*/)
            {
            }
            /**
             * @brief Starts a nested map in the configuration file
             *
             * @param[in] map_name name of the map
             */
            virtual void startMapEntry(const std::string &map_name)
            {
                ARILES2_UNUSED_ARG(map_name)
            }
            virtual void endMapEntry()
            {
            }
            /**
             * @brief Ends a nested map in the configuration file
             */
            virtual void endMap()
            {
            }


            virtual bool startIteratedMap(const std::size_t num_entries, const t_Parameters &param)
            {
                startMap(param, num_entries);
                return (true);
            }
            virtual void startIteratedMapElement(const std::string &map_name)
            {
                startMapEntry(map_name);
            }
            virtual void endIteratedMapElement()
            {
                endMapEntry();
            }
            virtual void endIteratedMap()
            {
                endMap();
            }


            virtual void startArray(const std::size_t size, const bool compact = false) = 0;
            virtual void startArrayElement()
            {
                ARILES2_TRACE_FUNCTION;
            }
            virtual void endArrayElement()
            {
                ARILES2_TRACE_FUNCTION;
            }
            virtual void endArray()
            {
                ARILES2_TRACE_FUNCTION;
            }


            virtual void startVector(const std::size_t size)
            {
                ARILES2_TRACE_FUNCTION;
                this->startArray(size, /*compact*/ true);
            }
            virtual void startVectorElement()
            {
                ARILES2_TRACE_FUNCTION;
                this->startArrayElement();
            }
            virtual void endVectorElement()
            {
                ARILES2_TRACE_FUNCTION;
                this->endArrayElement();
            }
            virtual void endVector()
            {
                ARILES2_TRACE_FUNCTION;
                this->endArray();
            }


            virtual void startMatrix(
                    const bool dynamic,
                    const std::size_t cols,
                    const std::size_t rows,
                    const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                if (param.flat_matrices_)
                {
                    if (dynamic or param.explicit_matrix_size_)
                    {
                        this->startMap(param, 3);

                        this->startMapEntry("cols");
                        this->writeElement(cols, param);
                        this->endMapEntry();

                        this->startMapEntry("rows");
                        this->writeElement(rows, param);
                        this->endMapEntry();

                        this->startMapEntry("data");
                    }

                    this->startVector(cols * rows);
                }
                else
                {
                    this->startArray(rows, /*compact=*/true);
                }
            }
            virtual void startMatrixRow(const std::size_t cols, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                if (not param.flat_matrices_)
                {
                    this->startArrayElement();
                    this->startVector(cols);
                }
            }
            virtual void startMatrixElement()
            {
                ARILES2_TRACE_FUNCTION;
                this->startVectorElement();
            }
            virtual void endMatrixElement()
            {
                ARILES2_TRACE_FUNCTION;
                this->endVectorElement();
            }
            virtual void endMatrixRow(const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                if (not param.flat_matrices_)
                {
                    this->endVector();
                    this->endArrayElement();
                }
            }
            virtual void endMatrix(const bool dynamic, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                if (param.flat_matrices_)
                {
                    this->endVector();

                    if (dynamic or param.explicit_matrix_size_)
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



            void startPointer(const bool is_null, const t_Parameters &param)
            {
                if (is_null)
                {
                    this->startMap(param, 1);
                    this->startMapEntry("is_null");
                    this->writeElement(is_null, param);
                    this->endMapEntry();
                }
                else
                {
                    this->startMap(param, 2);
                    this->startMapEntry("is_null");
                    this->writeElement(is_null, param);
                    this->endMapEntry();
                    this->startMapEntry("value");
                }
            }
            void endPointer(const bool is_null)
            {
                if (not is_null)
                {
                    this->endMapEntry();
                }
                this->endMap();
            }


            template <class t_Scalar>
            void writeElement(const std::complex<t_Scalar> &entry, const t_Parameters &param)
            {
                this->startArray(2, /*compact=*/true);
                this->startArrayElement();
                this->writeElement(entry.real(), param);
                this->endArrayElement();
                this->startArrayElement();
                this->writeElement(entry.imag(), param);
                this->endArrayElement();
                this->endArray();
            }
            virtual void writeElement(const std::complex<float> &entry, const t_Parameters &param)
            {
                writeElement<float>(entry, param);
            }
            virtual void writeElement(const std::complex<double> &entry, const t_Parameters &param)
            {
                writeElement<double>(entry, param);
            }


#define ARILES2_BASIC_TYPE(type) virtual void writeElement(const type &entry, const t_Parameters &param) = 0;

            ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE

            template <typename t_Entry>
            void visit(const t_Entry &entry, const std::string &entry_name, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                this->startRoot(entry_name, param);
                apply_write(static_cast<t_Derived &>(*this), entry, param);
                this->endRoot(entry_name);
                this->flush();
            }

            template <typename t_Entry>
            void visitMapEntry(const t_Entry &entry, const std::string &entry_name, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(entry_name);
                ARILES2_TRACE_TYPE(entry);

                if (param.allow_missing_entries_ and isMissing(entry))
                {
                    return;
                }

                this->startMapEntry(entry_name);
                apply_write(static_cast<t_Derived &>(*this), entry, param);
                this->endMapEntry();
            }

            template <typename t_Element>
            void visitArrayElement(const t_Element &element, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_TYPE(element);

                this->startArrayElement();
                apply_write(static_cast<t_Derived &>(*this), element, param);
                this->endArrayElement();
            }

            template <typename t_Element>
            void visitVectorElement(const t_Element &element, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;

                this->startVectorElement();
                this->writeElement(element, param);
                this->endVectorElement();
            }

            template <typename t_Element>
            void visitMatrixElement(const t_Element &element, const t_Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;

                this->startMatrixElement();
                this->writeElement(element, param);
                this->endMatrixElement();
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public VisitorBase<Visitor, Parameters>
        {
        protected:
            Visitor()
            {
            }
            ~Visitor()
            {
            }

        public:
            template <class t_Entry>
            void startMap(t_Entry &entry, const Parameters &parameters)
            {
                std::size_t map_size = ariles2::apply<ariles2::Count>(entry);
                if (parameters.allow_missing_entries_)
                {
                    map_size -= ariles2::apply<ariles2::CountMissing>(entry);
                }
                startMap(parameters, map_size);
            }

            using write::VisitorBase<Visitor, Parameters>::startMap;
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::ConstBase<write::Visitor>
        {
        };


#define ARILES2_NAMED_ENTRY_write(v, entry, name) visitor.visitMapEntry(entry, #name, parameters);
#define ARILES2_PARENT_write(v, entry)
#define ARILES2_VISIT_write                                                                                            \
    template <class t_Visitor>                                                                                         \
    void arilesVisit(                                                                                                  \
            t_Visitor &visitor,                                                                                        \
            const typename t_Visitor::Parameters &parameters,                                                          \
            ARILES2_IS_BASE_ENABLER(ariles2::write::Visitor, t_Visitor)) const                                         \
    {                                                                                                                  \
        ARILES2_TRACE_FUNCTION;                                                                                        \
        ARILES2_UNUSED_ARG(visitor);                                                                                   \
        ARILES2_UNUSED_ARG(parameters);                                                                                \
        arilesVisitParents(visitor, parameters);                                                                       \
        ARILES2_ENTRIES(write)                                                                                         \
    }

#define ARILES2_METHODS_write ARILES2_METHODS(write, ARILES2_EMPTY_MACRO, const)
#define ARILES2_BASE_METHODS_write ARILES2_BASE_METHODS(write)
    }  // namespace write


    /// @ingroup write
    using Write = write::Visitor;
}  // namespace ariles2
