/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp
        {
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer :
                public ariles::bridge::yaml_cpp::Base<ariles::WriterBase>
            {
                protected:
                    /// output file stream
                    std::ofstream   config_ofs_;

                    /// output stream
                    std::ostream    *output_stream_;

                    /// instance of YAML emitter, is destroyed and reinitialized by flush()
                    YAML::Emitter   *emitter_;


                protected:
                    /**
                     * @brief Initialize emitter
                     */
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


                    /**
                     * @brief Destroy emitter
                     */
                    void destroyEmitter()
                    {
                        *emitter_ << YAML::EndMap;
                        *output_stream_ << emitter_->c_str();
                        delete emitter_;
                    }


                public:
                    explicit Writer(const std::string& file_name)
                    {
                        WriterBase::openFile(config_ofs_, file_name);
                        output_stream_ = &config_ofs_;
                        initEmitter();
                    }


                    explicit Writer(std::ostream& output_stream)
                    {
                        output_stream_ = &output_stream;
                        initEmitter();
                    }


                    ~Writer()
                    {
                        delete emitter_;
                    }



                    void descend(const std::string &map_name)
                    {
                        *emitter_ << YAML::Key << map_name;
                        *emitter_ << YAML::Value;
                    }


                    void startMap(const std::size_t /*num_entries*/)
                    {
                        *emitter_ << YAML::BeginMap;
                    }


                    void endMap()
                    {
                        *emitter_ << YAML::EndMap;
                    }


                    void flush()
                    {
                        destroyEmitter();
                        *output_stream_ << "\n";
                        output_stream_->flush();
                        initEmitter();
                    }



                    void startArray(const std::size_t /*size*/, const bool compact = false)
                    {
                        if (true == compact)
                        {
                            *emitter_ << YAML::Flow;
                        }
                        *emitter_ << YAML::BeginSeq;
                    }


                    void endArray()
                    {
                        *emitter_ << YAML::EndSeq;
                    }



                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                *emitter_ << element; \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_INTEGER_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE


                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                if (true == ariles::isNaN(element)) \
                                { \
                                    *emitter_ << ".nan"; \
                                } \
                                else \
                                { \
                                    if (true == ariles::isInfinity(element)) \
                                    { \
                                        if (element < 0.0) \
                                        { \
                                            *emitter_ << "-.inf"; \
                                        } \
                                        else \
                                        { \
                                            *emitter_ << ".inf"; \
                                        } \
                                    } \
                                    else \
                                    { \
                                        *emitter_ << element; \
                                    } \
                                } \
                            }

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_REAL_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE



                    void writeElement(const std::string & element)
                    {
                        *emitter_ << element;
                    }

                    void writeElement(const bool & element)
                    {
                        *emitter_ << element;
                    }
            };
        }
    }
}
