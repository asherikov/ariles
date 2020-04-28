/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles/visitors/octave.h>

#include <limits>
#include <iomanip>
#include <vector>
#include <boost/lexical_cast.hpp>


namespace ariles
{
    namespace ns_octave
    {
        typedef ariles::Node<std::string> NodeWrapper;
    }
}  // namespace ariles

namespace ariles
{
    namespace ns_octave
    {
        namespace impl
        {
            class ARILES_LIB_LOCAL Writer
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
                void initEmitter()
                {
                    *output_stream_ << std::setprecision(std::numeric_limits<double>::digits10);
                }


            public:
                Writer(const std::string &file_name)
                {
                    ariles::write::Visitor::openFile(config_ofs_, file_name);
                    output_stream_ = &config_ofs_;
                    initEmitter();
                }

                Writer(std::ostream &output_stream)
                {
                    output_stream_ = &output_stream;
                    initEmitter();
                }
            };
        }  // namespace impl
    }      // namespace ns_octave
}  // namespace ariles

namespace ariles
{
    namespace ns_octave
    {
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }


        const serialization::Features &Writer::getSerializationFeatures() const
        {
            static serialization::Features parameters(serialization::Features::NATIVE_MATRIX_SUPPORTED);
            return (parameters);
        }


        void Writer::flush()
        {
            impl_->output_stream_->flush();
        }



        void Writer::descend(const std::string &map_name)
        {
            if (0 == impl_->node_stack_.size())
            {
                impl_->node_stack_.push_back(map_name);
            }
            else
            {
                if (true == impl_->node_stack_.back().isArray())
                {
                    std::string node = impl_->node_stack_.back().node_;
                    node += "{";
                    node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_ + 1);
                    node += "}.";
                    node += map_name;
                    impl_->node_stack_.push_back(NodeWrapper(node));
                }
                else
                {
                    impl_->node_stack_.push_back(impl_->node_stack_.back().node_ + "." + map_name);
                }
            }
        }

        void Writer::ascend()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startArray(const std::size_t size, const bool compact)
        {
            if (true == impl_->node_stack_.back().isArray())
            {
                std::string node = impl_->node_stack_.back().node_;
                node += "{";
                node += boost::lexical_cast<std::string>(impl_->node_stack_.back().index_ + 1);
                node += "}";
                impl_->node_stack_.push_back(NodeWrapper(node, 0, size, compact));
            }
            else
            {
                impl_->node_stack_.push_back(NodeWrapper(impl_->node_stack_.back().node_, 0, size));
            }
        }

        void Writer::shiftArray()
        {
            ARILES_ASSERT(true == impl_->node_stack_.back().isArray(), "Internal error: array expected.");
            ++impl_->node_stack_.back().index_;
        }

        void Writer::endArray()
        {
            impl_->node_stack_.pop_back();
        }


        void Writer::startMatrix(const bool compact)
        {
            std::string node = impl_->node_stack_.back().node_;
            if (compact)
            {
                node += " = [";
            }
            else
            {
                node += " = [...\n";
            }

            impl_->node_stack_.push_back(NodeWrapper(node, NodeWrapper::MATRIX, compact));
            *impl_->output_stream_ << impl_->node_stack_.back().node_;
        }

        void Writer::startMatrixRow()
        {
            impl_->node_stack_.back().index_ = 0;
        }

        void Writer::endMatrixRow()
        {
            if (impl_->node_stack_.back().isCompact())
            {
                *impl_->output_stream_ << "; ";
            }
            else
            {
                *impl_->output_stream_ << "; ...\n";
            }
        }

        void Writer::endMatrix()
        {
            *impl_->output_stream_ << "];\n";
            impl_->node_stack_.pop_back();
        }


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        if (true == impl_->node_stack_.back().isMatrix())                                                              \
        {                                                                                                              \
            if (0 != impl_->node_stack_.back().index_)                                                                 \
            {                                                                                                          \
                *impl_->output_stream_ << ", ";                                                                        \
            }                                                                                                          \
            *impl_->output_stream_ << element;                                                                         \
            ++impl_->node_stack_.back().index_;                                                                        \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            *impl_->output_stream_ << impl_->node_stack_.back().node_;                                                 \
            if (true == impl_->node_stack_.back().isArray())                                                           \
            {                                                                                                          \
                *impl_->output_stream_ << "{" << impl_->node_stack_.back().index_ + 1 << "}";                          \
            }                                                                                                          \
            *impl_->output_stream_ << " = " << element << ";\n";                                                       \
        }                                                                                                              \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES_BASIC_TYPE


        void Writer::writeElement(const std::string &element)
        {
            *impl_->output_stream_ << impl_->node_stack_.back().node_;
            if (true == impl_->node_stack_.back().isArray())
            {
                *impl_->output_stream_ << "{" << impl_->node_stack_.back().index_ + 1 << "}";
            }
            *impl_->output_stream_ << " = '" << element << "';\n";
        }
    }  // namespace ns_octave
}  // namespace ariles
