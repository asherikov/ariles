/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "serialization.h"

namespace ariles2
{
    namespace write
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Visitor : public serialization::Base
        {
        protected:
            Visitor(){};
            ~Visitor(){};


        public:
            /**
             * @brief open configuration file
             *
             * @param[out] config_ofs
             * @param[in] file_name
             */
            static void openFile(std::ofstream &config_ofs, const std::string &file_name)
            {
                config_ofs.open(file_name.c_str());

                ARILES2_PERSISTENT_ASSERT(
                        true == config_ofs.good(),
                        std::string("Could not open configuration file for writing: ") + file_name.c_str());
            }


            /**
             * @brief Flush the configuration to the output
             */
            virtual void flush() = 0;


            /**
             * @brief Starts a nested map in the configuration file
             *
             * @param[in] map_name name of the map
             */
            virtual void descend(const std::string &map_name)
            {
                ARILES2_UNUSED_ARG(map_name)
            }
            virtual void ascend()
            {
            }


            /**
             * @brief Starts a nested map in the configuration file
             *
             * @param[in] num_entries number of child entries
             */
            virtual void startMap(const std::size_t num_entries)
            {
                ARILES2_UNUSED_ARG(num_entries)
            }

            /**
             * @brief Ends a nested map in the configuration file
             */
            virtual void endMap()
            {
            }


            virtual void startArray(const std::size_t size, const bool compact = false) = 0;
            virtual void shiftArray()
            {
            }
            virtual void endArray()
            {
            }

            virtual void startMatrix(const bool compact = false)
            {
                ARILES2_UNUSED_ARG(compact)
            }
            virtual void startMatrixRow()
            {
            }
            virtual void endMatrixRow()
            {
            }
            virtual void endMatrix()
            {
            }


            virtual void startRoot(const std::string &name)
            {
                ARILES2_TRACE_FUNCTION;
                if (false == name.empty())
                {
                    descend(name);
                }
            }
            virtual void endRoot(const std::string &name)
            {
                ARILES2_TRACE_FUNCTION;
                if (false == name.empty())
                {
                    ascend();
                }
            }


#define ARILES2_BASIC_TYPE(type) virtual void writeElement(const type &entry) = 0;

            ARILES2_BASIC_TYPES_LIST

#undef ARILES2_BASIC_TYPE


            template <typename t_Entry>
            void start(const t_Entry &entry, const std::string &entry_name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                this->startRoot(entry_name);
                apply_write(*this, entry, param);
                this->endRoot(entry_name);
                flush();
            }


            template <typename t_Entry>
            void operator()(const t_Entry &entry, const std::string &entry_name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_ENTRY(entry_name);
                ARILES2_TRACE_TYPE(entry);

                this->descend(entry_name);
                apply_write(*this, entry, param);
                this->ascend();
            }
        };



        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::ConstBase<write::Visitor>
        {
        };

#define ARILES2_VISIT_write
#define ARILES2_METHODS_write ARILES2_METHODS(write, ARILES2_EMPTY_MACRO, const)
#define ARILES2_BASE_METHODS_write ARILES2_BASE_METHODS(write)
    }  // namespace write


    typedef write::Visitor Write;
}  // namespace ariles2
