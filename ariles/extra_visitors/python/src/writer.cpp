/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2026 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/python.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <stack>

namespace ariles2
{
    namespace ns_python
    {
        namespace impl
        {
            using Parameters = write::Parameters;

            class Writer : public write::FileVisitorImplementation,
                           public serialization::NodeStackBase<serialization::Node<std::string>>
            {
            public:
                using NodeWrapper = serialization::Node<std::string>;

            public:
                template <class... t_Args>
                explicit Writer(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                }

                void clear()
                {
                    node_stack_.clear();
                }
            };  // end of Writer class
        }  // namespace impl


        // Implement the wrapper methods in the outer ns_python namespace
        Writer::Writer(const std::string &file_name)
        {
            makeImplPtr(file_name);
        }


        Writer::Writer(std::ostream &output_stream)
        {
            makeImplPtr(output_stream);
        }


        void Writer::flush()
        {
            impl_->output_stream_->flush();
        }


        void Writer::startRoot(const std::string &name, const Parameters &)
        {
            CPPUT_TRACE_FUNCTION;
            impl_->clear();

            if (name.empty())
            {
                impl_->emplace("ariles");
            }
            else
            {
                impl_->emplace(name);
            }

            *impl_->output_stream_                                               //
                    << std::setprecision(std::numeric_limits<double>::digits10)  //
                    << "import numpy\n"                                          //
                    << "\n"                                                      //
                    << impl_->back().node_ << " = ";
        }


        void Writer::startMap(const Parameters &, const std::size_t)
        {
            *impl_->output_stream_ << "{";
        }

        void Writer::startMapEntry(const std::string &map_name)
        {
            *impl_->output_stream_ << "'" << map_name << "':";
            impl_->emplace(map_name);
        }


        void Writer::endMapEntry()
        {
            *impl_->output_stream_ << ",";
            impl_->pop();
        }

        void Writer::endMap()
        {
            *impl_->output_stream_ << "}";
        }


        void Writer::startArray(const std::size_t size, const bool)
        {
            *impl_->output_stream_ << "[";
            impl_->emplace(impl_->back().node_, 0, size);
        }

        void Writer::endArray()
        {
            *impl_->output_stream_ << "]";
            impl_->pop();
        }


        void Writer::endArrayElement()
        {
            impl_->shiftArray();
            *impl_->output_stream_ << ",";
        }


        void Writer::startVector(const std::size_t)
        {
            *impl_->output_stream_ << "numpy.array([";
            impl_->emplace(impl_->back().node_, impl::Writer::NodeWrapper::Type::VECTOR);
        }

        void Writer::endVectorElement()
        {
            *impl_->output_stream_ << ",";
        }

        void Writer::endVector()
        {
            *impl_->output_stream_ << "])";  // Close the numpy array
            impl_->pop();
        }


        void Writer::startMatrix(
                const bool,
                const std::size_t,
                const std::size_t,
                const Parameters &)
        {
            *impl_->output_stream_ << "numpy.array([";
            impl_->emplace(impl_->back().node_, impl::Writer::NodeWrapper::Type::MATRIX);
        }

        void Writer::endMatrix(const bool, const Parameters &)
        {
            *impl_->output_stream_ << "])";  // Close the numpy matrix
            impl_->pop();
        }


        void Writer::startMatrixRow(const std::size_t, const Parameters &)
        {
            *impl_->output_stream_ << "[";
        }

        void Writer::endMatrixRow(const Parameters &)
        {
            *impl_->output_stream_ << "],";
        }


        void Writer::endMatrixElement()
        {
            *impl_->output_stream_ << ",";
        }



#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        *impl_->output_stream_ << element;                                                                             \
    }

        ARILES2_BASIC_INTEGER_TYPES_LIST

#undef ARILES2_BASIC_TYPE

#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        if (std::isnan(element))                                                                                       \
        {                                                                                                              \
            *impl_->output_stream_ << "float('nan')";                                                                  \
        }                                                                                                              \
        else if (std::isinf(element))                                                                                  \
        {                                                                                                              \
            *impl_->output_stream_ << (element > 0 ? "float('inf')" : "float('-inf')");                                \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            *impl_->output_stream_ << element;                                                                         \
        }                                                                                                              \
    }

        ARILES2_BASIC_REAL_TYPES_LIST

#undef ARILES2_BASIC_TYPE

        void Writer::writeElement(const bool &element, const Parameters &)
        {
            *impl_->output_stream_ << (element ? "True" : "False");
        }

        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            *impl_->output_stream_ << "'" << element << "'";
        }
    }  // namespace ns_python
}  // namespace ariles2
