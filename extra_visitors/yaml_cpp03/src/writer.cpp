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

#include <ariles/visitors/yaml_cpp03.h>
#include "common.h"


namespace ariles
{
    namespace ns_yaml_cpp03
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


            protected:
                void initEmitter()
                {
                    emitter_ = new YAML::Emitter;
                    emitter_->SetDoublePrecision(std::numeric_limits<double>::digits10);
                    if (output_stream_->tellp() != 0)
                    {
                        *emitter_ << YAML::Newline;
                    }
                    *emitter_ << YAML::BeginMap;
                }


                void destroyEmitter()
                {
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
    }      // namespace ns_yaml_cpp03
}  // namespace ariles


namespace ariles
{
    namespace ns_yaml_cpp03
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
            *impl_->emitter_ << YAML::Key << map_name;
            *impl_->emitter_ << YAML::Value;
        }


        void Writer::startMap(const std::size_t /*num_entries*/)
        {
            *impl_->emitter_ << YAML::BeginMap;
        }


        void Writer::endMap()
        {
            *impl_->emitter_ << YAML::EndMap;
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


#define ARILES_BASIC_TYPE(type)                                                                    \
    void Writer::writeElement(const type &element)                                                 \
    {                                                                                              \
        *impl_->emitter_ << element;                                                               \
    }

        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_INTEGER_TYPES_LIST)

#undef ARILES_BASIC_TYPE


#define ARILES_BASIC_TYPE(type)                                                                    \
    void Writer::writeElement(const type &element)                                                 \
    {                                                                                              \
        if (true == ariles::isNaN(element))                                                        \
        {                                                                                          \
            *impl_->emitter_ << ".nan";                                                            \
        }                                                                                          \
        else                                                                                       \
        {                                                                                          \
            if (true == ariles::isInfinity(element))                                               \
            {                                                                                      \
                if (element < 0.0)                                                                 \
                {                                                                                  \
                    *impl_->emitter_ << "-.inf";                                                   \
                }                                                                                  \
                else                                                                               \
                {                                                                                  \
                    *impl_->emitter_ << ".inf";                                                    \
                }                                                                                  \
            }                                                                                      \
            else                                                                                   \
            {                                                                                      \
                *impl_->emitter_ << element;                                                       \
            }                                                                                      \
        }                                                                                          \
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
    }  // namespace ns_yaml_cpp03
}  // namespace ariles
