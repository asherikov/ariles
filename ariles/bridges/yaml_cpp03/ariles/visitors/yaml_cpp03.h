/**
    @file
    @author Alexander Sherikov
    @author Jan Michalczyk

    @copyright 2014-2017 INRIA. Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/visitors/config.h>

namespace ariles
{
    namespace bridge
    {
        namespace yaml_cpp03
        {
            template <class t_Base, class t_Implementation>
                class Base : public t_Base
            {
                protected:
                    typedef t_Implementation Impl;
                    typedef ARILES_SHARED_PTR<t_Implementation> ImplPtr;

                protected:
                    ImplPtr impl_;


                private:
                    Base(const Base&);
                    Base& operator=(const Base&);

                protected:
                    Base(){};
                    ~Base(){};


                public:
                    const BridgeFlags &getBridgeFlags() const
                    {
                        static BridgeFlags parameters(
                                BridgeFlags::SLOPPY_MAPS_SUPPORTED
                                | BridgeFlags::SLOPPY_PAIRS_SUPPORTED);
                        return (parameters);
                    }
            };
        }
    }
}


#include "./yaml_cpp03/reader.h"
#include "./yaml_cpp03/writer.h"

#define ARILES_BRIDGE_INCLUDED_yaml_cpp03


namespace ariles
{
    /**
     * @brief YAML bridge.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE yaml_cpp03
    {
        typedef ariles::cfgread::Visitor<bridge::yaml_cpp03::Reader> Reader;
        typedef ariles::cfgwrite::Visitor<bridge::yaml_cpp03::Writer> Writer;
    };
}
