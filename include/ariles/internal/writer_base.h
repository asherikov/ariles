/**
    @file
    @author  Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    class ARILES_VISIBILITY_ATTRIBUTE WriterBase
    {
        protected:
            /**
             * @brief open configuration file
             *
             * @param[out] config_ifs
             * @param[in] file_name
             */
            void openFile(std::ofstream &config_ofs, const std::string& file_name)
            {
                config_ofs.open(file_name.c_str());

                if (!config_ofs.good())
                {
                    ARILES_THROW_MSG(std::string("Could not open configuration file for writing: ") +  file_name.c_str());
                }
            }
    };


    class SloppyMapWriterBase
    {
    };
}
