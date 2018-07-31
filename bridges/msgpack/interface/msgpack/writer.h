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
            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer : public ariles::WriterBase
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


                    const BridgeParameters &getBridgeParameters() const
                    {
                        static BridgeParameters parameters(false);
                        return (parameters);
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



                    /**
                     * @brief Destructor
                     */
                    ~Writer()
                    {
                        delete packer_;
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     */
                    void descend(const std::string &map_name)
                    {
                        packer_->pack(map_name);
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     *
                     * @param[in] map_name name of the map
                     * @param[in] num_entries number of child entries
                     */
                    void startMap(const std::size_t num_entries)
                    {
                        packer_->pack_map(num_entries);
                    }


                    /**
                     * @brief Starts a nested map in the configuration file
                     */
                    void initRoot()
                    {
                        packer_->pack_map(1);
                    }


                    /**
                     * @brief Flush the configuration to the file
                     */
                    void flush()
                    {
                        output_stream_->flush();
                    }


                    void startArray(const std::size_t size, const bool /*compact*/ = false)
                    {
                        ARILES_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");

                        packer_->pack_array(size);
                    }


                    /**
                     * @brief Write a configuration entry (scalar template)
                     *
                     * @tparam t_EntryType type of the entry
                     *
                     * @param[in] entry      data
                     */

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
