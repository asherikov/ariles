/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

namespace ariles
{
    namespace write
    {
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::VisitorBase<ariles::ConfigurableFlags>
        {
            public:
                typedef ariles::ConfigurableFlags Parameters;


            protected:
                /**
                 * @brief open configuration file
                 *
                 * @param[out] config_ofs
                 * @param[in] file_name
                 */
                void openFile(std::ofstream &config_ofs, const std::string& file_name)
                {
                    config_ofs.open(file_name.c_str());

                    ARILES_PERSISTENT_ASSERT(   true == config_ofs.good(),
                                            std::string("Could not open configuration file for writing: ") + file_name.c_str());
                }


            public:
                using visitor::VisitorBase<Parameters>::getDefaultParameters;

                template<class t_Ariles>
                    const Parameters & getParameters(const t_Ariles & ariles_class) const
                {
                    return (ariles_class.arilesGetParameters(*this));
                }


                template<class t_Ariles>
                    void startRoot(const t_Ariles &, const Parameters &)
                {
                    ARILES_TRACE_FUNCTION;
                    initRoot();
                }

                template<class t_Ariles>
                    void finishRoot(const t_Ariles &, const Parameters &)
                {
                    ARILES_TRACE_FUNCTION;
                    flush();
                }


                virtual const BridgeFlags & getBridgeFlags() const = 0;

                /**
                 * @brief Starts a nested map in the configuration file
                 */
                virtual void initRoot() {}

                /**
                 * @brief Flush the configuration to the output
                 */
                virtual void flush() = 0;


                /**
                 * @brief Starts a nested map in the configuration file
                 *
                 * @param[in] map_name name of the map
                 */
                virtual void descend(const std::string &map_name) {ARILES_UNUSED_ARG(map_name)}
                virtual void ascend() {}


                /**
                 * @brief Starts a nested map in the configuration file
                 *
                 * @param[in] num_entries number of child entries
                 */
                virtual void startMap(const std::size_t num_entries) {ARILES_UNUSED_ARG(num_entries)}

                /**
                 * @brief Ends a nested map in the configuration file
                 */
                virtual void endMap() {}


                virtual void startArray(const std::size_t size, const bool compact = false) = 0;
                virtual void shiftArray() {}
                virtual void endArray() {}

                virtual void startMatrix(const bool compact = false) {ARILES_UNUSED_ARG(compact)}
                virtual void startMatrixRow() {}
                virtual void endMatrixRow() {}
                virtual void endMatrix() {}


                #define ARILES_BASIC_TYPE(type) \
                        virtual void writeElement(const type &entry) = 0;

                ARILES_BASIC_TYPES_LIST

                #undef ARILES_BASIC_TYPE


                template <typename t_Entry>
                    void operator()(
                            const t_Entry & entry,
                            const std::string & entry_name,
                            const Parameters & param)
                {
                    ARILES_TRACE_FUNCTION;
                    ARILES_TRACE_ENTRY(entry_name);
                    ARILES_TRACE_TYPE(entry);

                    this->descend(entry_name);
                    apply_write(*this, entry, param);
                    this->ascend();
                }
        };



        class ARILES_VISIBILITY_ATTRIBUTE Base
            : public visitor::ConstBase<write::Visitor>
        {
        };

#ifndef ARILES_METHODS_write
#   define ARILES_METHODS_write ARILES_METHODS(write, ariles::write::Visitor, const)
#endif
    }


    typedef write::Visitor Write;
}
