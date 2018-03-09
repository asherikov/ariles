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
    namespace yaml_cpp
    {
        /**
         * @brief Configuration writer class
         */
        class ARILES_VISIBILITY_ATTRIBUTE Writer
        {
            protected:
                /// output file stream
                std::ofstream   config_ofs_;

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
                    if (config_ofs_.tellp() != 0)
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
                    config_ofs_ << emitter_->c_str();
                    delete emitter_;
                }


            public:
                explicit Writer(const std::string& file_name)
                {
                    config_ofs_.open(file_name.c_str());

                    if (!config_ofs_.good())
                    {
                        ARILES_THROW_MSG(std::string("Could not open configuration file for writing: ") +  file_name.c_str());
                    }

                    initEmitter();
                }


                ~Writer()
                {
                    delete emitter_;
                }


                /**
                 * @brief Starts a nested map in the configuration file
                 *
                 * @param[in] map_name name of the map
                 */
                void descend(const std::string &map_name)
                {
                    *emitter_ << YAML::Key << map_name;
                    *emitter_ << YAML::Value;
                }


                /**
                 * @brief Starts a nested map in the configuration file
                 *
                 * @param[in] num_entries number of child entries
                 */
                void startMap(const std::size_t num_entries)
                {
                    ARILES_IGNORE_UNUSED(num_entries);
                    *emitter_ << YAML::BeginMap;
                }


                /**
                 * @brief Starts a nested map in the configuration file
                 */
                void initRoot()
                {
                }


                /**
                 * @brief Ends a nested map in the configuration file
                 */
                void endMap()
                {
                    *emitter_ << YAML::EndMap;
                }

                void ascend()
                {
                }



                /**
                 * @brief Flush the configuration to the file
                 */
                void flush()
                {
                    destroyEmitter();
                    initEmitter();
                }



                void startArray(const std::size_t size)
                {
                    ARILES_IGNORE_UNUSED(size);
                    *emitter_ << YAML::Flow;
                    *emitter_ << YAML::BeginSeq;
                }

                void shiftArray()
                {
                }

                void endArray()
                {
                    *emitter_ << YAML::EndSeq;
                }


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
                    *emitter_ << element;
                }
        };
    }
}