/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_VISITOR_INCLUDED_pugixml

#include <ariles2/internal/helpers.h>
#include <ariles2/internal/node.h>
#include <ariles2/visitors/config.h>


namespace ariles2
{
    namespace ns_pugixml
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
                static serialization::Features parameters(
                        serialization::Features::SLOPPY_MAPS_SUPPORTED
                        | serialization::Features::SLOPPY_PAIRS_SUPPORTED);
                return (parameters);
            }
        };
    }  // namespace ns_pugixml
}  // namespace ariles2



#include "./pugixml/reader.h"
#include "./pugixml/writer.h"


namespace ariles2
{
    /**
     * @brief pugixml visitor.
     */
    struct ARILES2_VISIBILITY_ATTRIBUTE pugixml
    {
        typedef ariles2::cfgread::Visitor<ns_pugixml::Reader> Reader;
        typedef ariles2::cfgwrite::Visitor<ns_pugixml::Writer> Writer;
    };
}  // namespace ariles2
