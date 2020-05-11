/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_VISITOR_INCLUDED_rapidjson

#include <ariles2/internal/helpers.h>
#include <ariles2/internal/node.h>
#include <ariles2/visitors/config.h>


namespace ariles
{
    namespace ns_rapidjson
    {
        class ARILES_VISIBILITY_ATTRIBUTE Flags : public ariles::Flags<unsigned int, Flags>
        {
        public:
            enum Enum
            {
                RESET = 0,
                /// Floats are stored as strings by default to allow NaN and Inf (writer only)
                DISABLE_STRING_FLOATS = 1 << 0,

                DEFAULT = RESET
            };


        public:
            Flags()
            {
                setDefaults();
            }


            Flags(const unsigned int flags, const Action action_type = REPLACE)
            {
                initialize(flags, action_type);
            }


            void setDefaults()
            {
                flags_ = DEFAULT;
            }
        };


        template <class t_Base, class t_Implementation>
        class ARILES_VISIBILITY_ATTRIBUTE Base : public t_Base
        {
        protected:
            typedef t_Implementation Impl;
            typedef ARILES_SHARED_PTR<t_Implementation> ImplPtr;

        protected:
            ImplPtr impl_;
            Flags flags_;


        private:
            Base(const Base &);
            Base &operator=(const Base &);

        protected:
            Base(){};
            ~Base(){};
            Base(const Flags &flags) : flags_(flags){};


        public:
            const serialization::Features &getSerializationFeatures() const
            {
                static serialization::Features parameters(
                        serialization::Features::SLOPPY_MAPS_SUPPORTED
                        | serialization::Features::SLOPPY_PAIRS_SUPPORTED);
                return (parameters);
            }
        };
    }  // namespace ns_rapidjson
}  // namespace ariles


#include "./rapidjson/reader.h"
#include "./rapidjson/writer.h"


namespace ariles
{
    /**
     * @brief JSON visitor.
     */
    struct ARILES_VISIBILITY_ATTRIBUTE rapidjson
    {
        typedef ariles::ns_rapidjson::Flags Flags;

        typedef ns_rapidjson::Reader ReaderBase;
        typedef ns_rapidjson::Writer WriterBase;

        typedef ariles::cfgread::Visitor<ns_rapidjson::Reader> Reader;
        typedef ariles::cfgwrite::Visitor<ns_rapidjson::Writer> Writer;
    };
}  // namespace ariles
