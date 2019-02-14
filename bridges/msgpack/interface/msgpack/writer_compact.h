/**
    @file
    @author Alexander Sherikov

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    namespace bridge
    {
        namespace msgpack
        {
            namespace compact
            {
                /**
                 * @brief Configuration writer class
                 */
                class ARILES_VISIBILITY_ATTRIBUTE Writer :
                    public ariles::bridge::msgpack::Base<ariles::WriterBase>
                {
                    protected:
                        /// output file stream
                        std::ofstream   config_ofs_;

                        /// output stream
                        std::ostream    *output_stream_;

                        ::msgpack::packer< std::ostream > *packer_;


                    public:
                        /**
                         * @brief Constructor
                         *
                         * @param[in] file_name
                         */
                        explicit Writer(const std::string& file_name)
                        {
                            WriterBase::openFile(config_ofs_, file_name);
                            output_stream_ = &config_ofs_;
                            packer_ = new ::msgpack::packer< std::ostream >(*output_stream_);
                        }


                        /**
                         * @brief Constructor
                         *
                         * @param[out] output_stream
                         */
                        explicit Writer(std::ostream& output_stream)
                        {
                            output_stream_ = &output_stream;
                            packer_ = new ::msgpack::packer< std::ostream >(*output_stream_);
                        }



                        ~Writer()
                        {
                            delete packer_;
                        }



                        void startMap(const std::size_t num_entries)
                        {
                            packer_->pack_array(num_entries);
                        }


                        void flush()
                        {
                            output_stream_->flush();
                        }


                        void startArray(const std::size_t size, const bool /*compact*/ = false)
                        {
                            ARILES_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");
                            packer_->pack_array(size);
                        }


                        #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element) \
                            { \
                                packer_->pack(element); \
                            }

                        ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                        #undef ARILES_BASIC_TYPE
                };
            }
        }
    }
}
