/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles
{
    template <class t_RawNode>
        class ARILES_VISIBILITY_ATTRIBUTE Node
    {
        public:
            const t_RawNode     *node_;
            std::size_t         index_;
            std::size_t         size_;

        public:
            Node(const t_RawNode *node) : node_(node)
            {
                index_ = 0;
                size_ = 0;
            }

            Node(const std::size_t index, const std::size_t size) : index_(index)
            {
                node_ = NULL;
                size_ = size;
            }

            bool isArray() const
            {
                return(NULL == node_);
            }
    };


    class ARILES_VISIBILITY_ATTRIBUTE ReaderBase
    {
        protected:
            /**
             * @brief open configuration file
             *
             * @param[out] config_ifs_
             * @param[in] file_name
             */
            void openFile(std::ifstream &config_ifs_, const std::string& file_name)
            {
                config_ifs_.open(file_name.c_str());
                if (!config_ifs_.good())
                {
                    std::string file_name_default = std::string(ARILES_DEFAULT_CONFIG_PREFIX) + file_name;
                    config_ifs_.open(file_name_default.c_str());
                }
                if (!config_ifs_.good())
                {
                    ARILES_THROW_MSG(std::string("Could not open configuration file: ") +  file_name.c_str());
                }
            }
    };
}
