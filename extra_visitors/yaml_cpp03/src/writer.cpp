/**
    @file
    @author Jan Michalczyk
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles2/visitors/yaml_cpp03.h>
#include "common.h"


namespace ariles2
{
    namespace ns_yaml_cpp03
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_ATTRIBUTE Writer
            {
            public:
                typedef ARILES2_SHARED_PTR<YAML::Emitter> EmitterPtr;


            public:
                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;

                /// instance of YAML emitter, is destroyed and reinitialized by flush()
                EmitterPtr emitter_;

                std::size_t map_depth_;
                bool skip_root_map_;


            protected:
                void initEmitter()
                {
                    emitter_ = EmitterPtr(new YAML::Emitter);
                    emitter_->SetDoublePrecision(std::numeric_limits<double>::digits10);
                    if (output_stream_->tellp() != 0)
                    {
                        *emitter_ << YAML::Newline;
                    }
                    *emitter_ << YAML::BeginMap;
                    map_depth_ = 0;
                    skip_root_map_ = false;
                }


                void destroyEmitter()
                {
                    *emitter_ << YAML::EndMap;
                    *output_stream_ << emitter_->c_str();
                    emitter_.reset();
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


                void flush()
                {
                    destroyEmitter();
                    *output_stream_ << "\n";
                    output_stream_->flush();
                    initEmitter();
                }
            };
        }  // namespace impl
    }      // namespace ns_yaml_cpp03
}  // namespace ariles2


namespace ariles2
{
    namespace ns_yaml_cpp03
    {
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new impl::Writer(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new impl::Writer(output_stream));
        }



        void Writer::startMap(const std::string & /*id*/, const std::size_t /*num_entries*/)
        {
            if (impl_->map_depth_ > 0 or false == impl_->skip_root_map_)
            {
                *impl_->emitter_ << YAML::BeginMap;
            }
            ++impl_->map_depth_;
        }

        void Writer::startMapElement(const std::string &map_name)
        {
            *impl_->emitter_ << YAML::Key << map_name;
            *impl_->emitter_ << YAML::Value;
        }

        void Writer::endMap()
        {
            ARILES2_ASSERT(impl_->map_depth_ > 0, "Internal logic error.");
            --impl_->map_depth_;
            if (impl_->map_depth_ > 0 or false == impl_->skip_root_map_)
            {
                *impl_->emitter_ << YAML::EndMap;
            }
        }


        void Writer::flush()
        {
            impl_->flush();
        }


        void Writer::startArray(const std::size_t /*size*/, const bool compact)
        {
            if (true == compact)
            {
                *impl_->emitter_ << YAML::Flow;
            }
            *impl_->emitter_ << YAML::BeginSeq;
        }

        void Writer::endArray()
        {
            *impl_->emitter_ << YAML::EndSeq;
        }


        void Writer::startRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (true == name.empty())
            {
                impl_->skip_root_map_ = true;
            }
            else
            {
                startMapElement(name);
            }
        }

        void Writer::endRoot(const std::string &name)
        {
            ARILES2_TRACE_FUNCTION;
            if (false == name.empty())
            {
                endMapElement();
            }
            impl_->skip_root_map_ = false;
        }


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        *impl_->emitter_ << element;                                                                                   \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_INTEGER_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


#define ARILES2_BASIC_TYPE(type)                                                                                       \
    void Writer::writeElement(const type &element, const Parameters &)                                                 \
    {                                                                                                                  \
        if (true == ariles2::isNaN(element))                                                                           \
        {                                                                                                              \
            *impl_->emitter_ << ".nan";                                                                                \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            if (true == ariles2::isInfinity(element))                                                                  \
            {                                                                                                          \
                if (element < 0.0)                                                                                     \
                {                                                                                                      \
                    *impl_->emitter_ << "-.inf";                                                                       \
                }                                                                                                      \
                else                                                                                                   \
                {                                                                                                      \
                    *impl_->emitter_ << ".inf";                                                                        \
                }                                                                                                      \
            }                                                                                                          \
            else                                                                                                       \
            {                                                                                                          \
                *impl_->emitter_ << element;                                                                           \
            }                                                                                                          \
        }                                                                                                              \
    }

        ARILES2_MACRO_SUBSTITUTE(ARILES2_BASIC_REAL_TYPES_LIST)

#undef ARILES2_BASIC_TYPE



        void Writer::writeElement(const std::string &element, const Parameters &)
        {
            *impl_->emitter_ << element;
        }

        void Writer::writeElement(const bool &element, const Parameters &)
        {
            *impl_->emitter_ << element;
        }
    }  // namespace ns_yaml_cpp03
}  // namespace ariles2
