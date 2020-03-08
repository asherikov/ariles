/**
    @file
    @author Alexander Sherikov

    @copyright 2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/visitors/config.h>

#define ARILES_BRIDGE_INCLUDED_octave


namespace ariles
{
    namespace bridge
    {
        namespace octave
        {
            namespace impl
            {
                class ARILES_LIB_LOCAL Writer;
            }


            /**
             * @brief Configuration writer class
             */
            class ARILES_VISIBILITY_ATTRIBUTE Writer : public ariles::write::Visitor
            {
                protected:
                    typedef impl::Writer Impl;
                    typedef ARILES_SHARED_PTR<impl::Writer> ImplPtr;
                    ImplPtr impl_;


                public:
                    explicit Writer(const std::string& file_name);
                    explicit Writer(std::ostream& output_stream);


                    const BridgeFlags &getBridgeFlags() const;


                    void flush();


                    void initRoot() {};


                    void descend(const std::string & map_name);
                    void ascend();


                    void startMap(const std::size_t /*num_entries*/) {}
                    void endMap() {}


                    void startArray(const std::size_t size, const bool compact = false);
                    void shiftArray();
                    void endArray();


                    void startMatrix(const bool compact = false);
                    void startMatrixRow();
                    void endMatrixRow();
                    void endMatrix();


                    #define ARILES_BASIC_TYPE(type) \
                            void writeElement(const type & element);

                    ARILES_MACRO_SUBSTITUTE(ARILES_BASIC_TYPES_LIST)

                    #undef ARILES_BASIC_TYPE
            };
        }
    }
}


namespace ariles
{
    /**
     * @brief Octave bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE octave
    {
        typedef ariles::cfgwrite::Visitor<bridge::octave::Writer> Writer;
    };
}
