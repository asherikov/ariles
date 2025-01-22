/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/octave.h>

#include <limits>
#include <iomanip>
#include <vector>
#include <boost/lexical_cast.hpp>


namespace ariles2
{
    namespace ns_octave
    {
        using NodeWrapper = serialization::Node<std::string>;
    }  // namespace ns_octave
}  // namespace ariles2


namespace ariles2
{
    namespace ns_octave
    {
        namespace impl
        {
            class Writer : public serialization::NodeStackBase<NodeWrapper>, public write::FileVisitorImplementation
            {
            public:
                const std::string separator_ = ".";
                const std::string bracket_left_ = "{";
                const std::string bracket_right_ = "}";


            protected:
                /**
                 * @brief Initialize emitter
                 */
                void initEmitter() const
                {
                    *output_stream_ << std::setprecision(std::numeric_limits<double>::digits10);
                }


            public:
                template <class... t_Args>
                explicit Writer(t_Args &&...args) : FileVisitorImplementation(std::forward<t_Args>(args)...)
                {
                    initEmitter();
                }

                template <typename t_Scalar>
                void writeComplex(const std::complex<t_Scalar> &element)
                {
                    if (back().isMatrix())
                    {
                        *output_stream_ << element.real() << " + " << element.imag() << "i";
                    }
                    else
                    {
                        *output_stream_ << back().node_;
                        if (back().isArray())
                        {
                            *output_stream_ << "{" << back().index_ + 1 << "}";
                        }
                        *output_stream_ << " = " << element.real() << " + " << element.imag() << "i"
                                        << ";\n";
                    }
                }
            };
        }  // namespace impl
    }      // namespace ns_octave
}  // namespace ariles2

namespace ariles2
{
    namespace ns_octave
    {
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



        void Writer::startMapEntry(const std::string &map_name)
        {
            if (impl_->empty())
            {
                impl_->emplace(map_name);
            }
            else
            {
                if (impl_->back().isArray())
                {
                    impl_->concatWithNodeAndEmplace(
                            impl_->bracket_left_,
                            boost::lexical_cast<std::string>(impl_->back().index_ + 1),
                            impl_->bracket_right_,
                            impl_->separator_,
                            map_name);
                }
                else
                {
                    impl_->concatWithNodeAndEmplace(impl_->separator_, map_name);
                }
            }
        }

        void Writer::endMapEntry()
        {
            impl_->pop();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            if (impl_->back().isArray())
            {
                impl_->emplace(
                        impl_->concatWithNode(
                                impl_->bracket_left_,
                                boost::lexical_cast<std::string>(impl_->back().index_ + 1),
                                impl_->bracket_right_),
                        0,
                        size);
            }
            else
            {
                impl_->emplace(impl_->back().node_, 0, size);
            }
        }

        void Writer::endArrayElement()
        {
            impl_->shiftArray();
        }

        void Writer::endArray()
        {
            impl_->pop();
        }


        void Writer::startVector(const std::size_t /*size*/)
        {
            impl_->emplace(impl_->back().node_ + " = [", NodeWrapper::Type::MATRIX);
            *impl_->output_stream_ << impl_->back().node_;
        }

        void Writer::startVectorElement()
        {
        }

        void Writer::endVectorElement()
        {
            *impl_->output_stream_ << "; ";
        }

        void Writer::endVector()
        {
            *impl_->output_stream_ << "];\n";
            impl_->pop();
        }


        void Writer::startMatrix(
                const bool /*dynamic*/,
                const std::size_t /*cols*/,
                const std::size_t /*rows*/,
                const Parameters & /*param*/)
        {
            impl_->emplace(impl_->back().node_ + " = [...\n", NodeWrapper::Type::MATRIX);
            *impl_->output_stream_ << impl_->back().node_;
        }

        void Writer::startMatrixRow(const std::size_t /*cols*/, const Parameters & /*param*/)
        {
            impl_->back().index_ = 0;
        }

        void Writer::startMatrixElement()
        {
            if (0 != impl_->back().index_)
            {
                *impl_->output_stream_ << ", ";
            }
        }

        void Writer::endMatrixElement()
        {
            ++impl_->back().index_;
        }

        void Writer::endMatrixRow(const Parameters & /*param*/)
        {
            *impl_->output_stream_ << "; ...\n";
        }

        void Writer::endMatrix(const bool /*dynamic*/, const Parameters & /*param*/)
        {
            *impl_->output_stream_ << "];\n";
            impl_->pop();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        if (impl_->back().isMatrix())                                                                                  \
        {                                                                                                              \
            *impl_->output_stream_ << element;                                                                         \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            *impl_->output_stream_ << impl_->back().node_;                                                             \
            if (impl_->back().isArray())                                                                               \
            {                                                                                                          \
                *impl_->output_stream_ << "{" << impl_->back().index_ + 1 << "}";                                      \
            }                                                                                                          \
            *impl_->output_stream_ << " = " << element << ";\n";                                                       \
        }                                                                                                              \
    }

        CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            *impl_->output_stream_ << impl_->back().node_;
            if (impl_->back().isArray())
            {
                *impl_->output_stream_ << "{" << impl_->back().index_ + 1 << "}";
            }
            *impl_->output_stream_ << " = '" << element << "';\n";
        }

        void Writer::writeElement(const std::complex<float> &element, const Parameters &)
        {
            impl_->writeComplex(element);
        }

        void Writer::writeElement(const std::complex<double> &element, const Parameters &)
        {
            impl_->writeComplex(element);
        }
    }  // namespace ns_octave
}  // namespace ariles2
