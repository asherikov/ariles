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

                    ::msgpack::packer< std::ofstream > *packer_;


                public:
                    /**
                     * @brief Constructor
                     *
                     * @param[in] file_name
                     */
                    explicit Writer(const std::string& file_name)
                    {
                        config_ofs_.open(file_name.c_str());

                        if (!config_ofs_.good())
                        {
                            ARILES_THROW_MSG(std::string("Could not open configuration file for writing: ") +  file_name.c_str());
                        }

                        packer_ = new ::msgpack::packer< std::ofstream >(config_ofs_);
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

                    void endMap()
                    {
                    }

                    /**
                     * @brief Starts a nested map in the configuration file
                     */
                    void initRoot()
                    {
                        packer_->pack_map(1);
                    }


                    /**
                     * @brief Ends a nested map in the configuration file
                     */
                    void ascend()
                    {
                    }


                    /**
                     * @brief Flush the configuration to the file
                     */
                    void flush()
                    {
                        config_ofs_.flush();
                    }


                    void startArray(const std::size_t size, const bool compact = false)
                    {
                        ARILES_IGNORE_UNUSED(compact);

                        ARILES_ASSERT(size <= std::numeric_limits<uint32_t>::max(), "Vector is too long.");

                        packer_->pack_array(size);
                    }

                    void shiftArray()
                    {
                    }

                    void endArray() const {}


                    /**
                     * @brief Write a configuration entry (scalar template)
                     *
                     * @tparam t_EntryType type of the entry
                     *
                     * @param[in] entry      data
                     */
                    template<class t_Element>
                        void writeElement(const t_Element & element)
                    {
                        packer_->pack(element);
                    }
            };
        }
    }
}
