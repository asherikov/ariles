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

#define ARILES_VISITOR_INCLUDED_msgpack

#include <ariles2/internal/helpers.h>
#include <ariles2/internal/node.h>
#include <ariles2/visitors/config.h>

namespace ariles2
{
    namespace ns_msgpack
    {
        template <class t_Base, class t_Implementation>
        class ARILES2_VISIBILITY_ATTRIBUTE Base : public t_Base
        {
        protected:
            typedef t_Implementation Impl;
            typedef ARILES_SHARED_PTR<t_Implementation> ImplPtr;

        protected:
            ImplPtr impl_;


        private:
            Base(const Base &);
            Base &operator=(const Base &);

        protected:
            Base(){};
            ~Base(){};


        public:
            const serialization::Features &getSerializationFeatures() const
            {
                static const serialization::Features parameters;  // all disabled
                return (parameters);
            }
        };
    }  // namespace ns_msgpack
}  // namespace ariles2



#include "./msgpack/reader.h"
#include "./msgpack/writer.h"
#include "./msgpack/reader_compact.h"
#include "./msgpack/writer_compact.h"


namespace ariles2
{
    /**
     * @brief MessagePack visitor.
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE msgpack
    {
        typedef ariles2::cfgread::Visitor<ns_msgpack::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_msgpack::Writer> Writer;
    };
}  // namespace ariles2

namespace ariles2
{
    struct ARILES2_VISIBILITY_ATTRIBUTE msgpack_compact
    {
        typedef ariles2::cfgread::Visitor<ns_msgpack_compact::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_msgpack_compact::Writer> Writer;
    };
}  // namespace ariles2
