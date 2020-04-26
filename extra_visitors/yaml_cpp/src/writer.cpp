/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#include <ariles/visitors/yaml_cpp.h>
#include <yaml-cpp/yaml.h>


namespace ariles
{
    namespace ns_yaml_cpp
    {
        namespace impl
        {
            class Writer
            {
            public:
                /// output file stream
                std::ofstream config_ofs_;

                /// output stream
                std::ostream *output_stream_;

                /// instance of YAML emitter, is destroyed and reinitialized by flush()
                YAML::Emitter *emitter_;

                std::size_t map_depth_;
                bool skip_root_map_;


            public:
                void initEmitter()
                {
                    ARILES_TRACE_FUNCTION;
                    emitter_ = new YAML::Emitter;
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
                    ARILES_TRACE_FUNCTION;
                    *emitter_ << YAML::EndMap;
                    *output_stream_ << emitter_->c_str();
                    delete emitter_;
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


                Writer()
                {
                    delete emitter_;
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
    }      // namespace ns_yaml_cpp
}  // namespace ariles


namespace ariles
{
    namespace ns_yaml_cpp
    {
        Writer::Writer(const std::string &file_name)
        {
            impl_ = ImplPtr(new Impl(file_name));
        }


        Writer::Writer(std::ostream &output_stream)
        {
            impl_ = ImplPtr(new Impl(output_stream));
        }



        void Writer::descend(const std::string &map_name)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(map_name);
            *impl_->emitter_ << YAML::Key << map_name;
            *impl_->emitter_ << YAML::Value;
        }


        void Writer::startMap(const std::size_t /*num_entries*/)
        {
            ARILES_TRACE_FUNCTION;
            if (impl_->map_depth_ > 0 or false == impl_->skip_root_map_)
            {
                *impl_->emitter_ << YAML::BeginMap;
            }
            ++impl_->map_depth_;
        }


        void Writer::endMap()
        {
            ARILES_TRACE_FUNCTION;
            ARILES_ASSERT(impl_->map_depth_ > 0, "Internal logic error.");
            --impl_->map_depth_;
            if (impl_->map_depth_ > 0 or false == impl_->skip_root_map_)
            {
                *impl_->emitter_ << YAML::EndMap;
            }
        }


        void Writer::flush()
        {
            ARILES_TRACE_FUNCTION;
            impl_->flush();
        }



        void Writer::startArray(const std::size_t /*size*/, const bool compact)
        {
            ARILES_TRACE_FUNCTION;
            if (true == compact)
            {
                *impl_->emitter_ << YAML::Flow;
            }
            *impl_->emitter_ << YAML::BeginSeq;
        }


        void Writer::endArray()
        {
            ARILES_TRACE_FUNCTION;
            *impl_->emitter_ << YAML::EndSeq;
        }


        void Writer::startRoot(const std::string &name)
        {
            ARILES_TRACE_FUNCTION;
            ARILES_TRACE_ENTRY(name);
            if (true == name.empty())
            {
                impl_->skip_root_map_ = true;
            }
            else
            {
                descend(name);
            }
        }

        void Writer::endRoot(const std::string &name)
        {
            ARILES_TRACE_FUNCTION;
            if (false == name.empty())
            {
                ascend();
            }
            impl_->skip_root_map_ = false;
        }


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        *impl_->emitter_ << element;                                                                                   \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE


#define ARILES_BASIC_TYPE(type)                                                                                        \
    void Writer::writeElement(const type &element)                                                                     \
    {                                                                                                                  \
        if (true == ariles::isNaN(element))                                                                            \
        {                                                                                                              \
            *impl_->emitter_ << ".nan";                                                                                \
        }                                                                                                              \
        else                                                                                                           \
        {                                                                                                              \
            if (true == ariles::isInfinity(element))                                                                   \
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

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

#undef ARILES_BASIC_TYPE



        void Writer::writeElement(const std::string &element)
        {
            *impl_->emitter_ << element;
        }

        void Writer::writeElement(const bool &element)
        {
            *impl_->emitter_ << element;
        }
    }  // namespace ns_yaml_cpp
}  // namespace ariles
