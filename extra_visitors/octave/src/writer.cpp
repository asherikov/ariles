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
            class ARILES2_VISIBILITY_ATTRIBUTE Writer
            {
            public:
                std::vector<NodeWrapper> node_stack_;

                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;


            protected:
                /**
                 * @brief Initialize emitter
                 */
                void initEmitter() const
                {
                    *output_stream_ << std::setprecision(std::numeric_limits<double>::digits10);
                }


            public:
                explicit Writer(const std::string &file_name)
                {
                    ariles2::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                    initEmitter();
                }

                explicit Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                    initEmitter();
                }

                template <typename t_Scalar>
                void writeComplex(const std::complex<t_Scalar> &element)
                {
                    if (node_stack_.back().isMatrix())
                    {
                        *output_stream_ << element.real() << " + " << element.imag() << "i";
                    }
                    else
                    {
                        *output_stream_ << node_stack_.back().node_;
                        if (node_stack_.back().isArray())
                        {
                            *output_stream_ << "{" << node_stack_.back().index_ + 1 << "}";
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
            if (impl_->node_stack_.empty())
            {
                impl_->node_stack_.emplace_back(map_name);
            }
            else
            {
                std::string node;
                if (impl_->node_stack_.back().isArray())
                {
                    node.reserve(impl_->node_stack_.back().node_.size() + map_name.size() + 15);

                    node = impl_->node_stack_.back().node_;
                    node += "{";
                    node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_ + 1);
                    node += "}.";
                }
                else
                {
                    node.reserve(impl_->node_stack_.back().node_.size() + map_name.size() + 1);
                    node = impl_->node_stack_.back().node_;
                    node += ".";
                }
                node += map_name;
                impl_->node_stack_.emplace_back(std::move(node));
            }
        }

        void Writer::endMapEntry()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startArray(const std::size_t size, const bool /*compact*/)
        {
            if (impl_->node_stack_.back().isArray())
            {
                std::string node;
                node.reserve(impl_->node_stack_.back().node_.size() + 15);
                node = impl_->node_stack_.back().node_;
                node += "{";
                node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_ + 1);
                node += "}";
                impl_->node_stack_.emplace_back(std::move(node), 0, size);
            }
            else
            {
                impl_->node_stack_.emplace_back(impl_->node_stack_.back().node_, 0, size);
            }
        }

        void Writer::endArrayElement()
        {
            ARILES2_ASSERT(impl_->node_stack_.back().isArray(), "Internal error: array expected.");
            ++impl_->node_stack_.back().index_;
        }

        void Writer::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startVector(const std::size_t /*size*/)
        {
            impl_->node_stack_.emplace_back(impl_->node_stack_.back().node_ + " = [", NodeWrapper::MATRIX);
            *impl_->output_stream_ << impl_->node_stack_.back().node_;
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
            impl_->node_stack_.pop_back();
        }


        void Writer::startMatrix(
                const bool /*dynamic*/,
                const std::size_t /*cols*/,
                const std::size_t /*rows*/,
                const Parameters & /*param*/)
        {
            impl_->node_stack_.emplace_back(impl_->node_stack_.back().node_ + " = [...\n", NodeWrapper::MATRIX);
            *impl_->output_stream_ << impl_->node_stack_.back().node_;
        }

        void Writer::startMatrixRow(const std::size_t /*cols*/, const Parameters & /*param*/)
        {
            impl_->node_stack_.back().index_ = 0;
        }

        void Writer::startMatrixElement()
        {
            if (0 != impl_->node_stack_.back().index_)
            {
                *impl_->output_stream_ << ", ";
            }
        }

        void Writer::endMatrixElement()
        {
            ++impl_->node_stack_.back().index_;
        }

        void Writer::endMatrixRow(const Parameters & /*param*/)
        {
            *impl_->output_stream_ << "; ...\n";
        }

        void Writer::endMatrix(const bool /*dynamic*/, const Parameters & /*param*/)
        {
            *impl_->output_stream_ << "];\n";
            impl_->node_stack_.pop_back();
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        if (impl_->node_stack_.back().isMatrix())                                                                      \
        {                                                                                                              \
            *impl_->output_stream_ << element;                                                                         \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            *impl_->output_stream_ << impl_->node_stack_.back().node_;                                                 \
            if (impl_->node_stack_.back().isArray())                                                                   \
            {                                                                                                          \
                *impl_->output_stream_ << "{" << impl_->node_stack_.back().index_ + 1 << "}";                          \
            }                                                                                                          \
            *impl_->output_stream_ << " = " << element << ";\n";                                                       \
        }                                                                                                              \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            *impl_->output_stream_ << impl_->node_stack_.back().node_;
            if (impl_->node_stack_.back().isArray())
            {
                *impl_->output_stream_ << "{" << impl_->node_stack_.back().index_ + 1 << "}";
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
