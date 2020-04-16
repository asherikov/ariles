/**
    @file
    @author Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_VISITOR_INCLUDED_rapidjson

#include <ariles/internal/helpers.h>
#include <ariles/internal/node.h>
#include <ariles/visitors/config.h>


namespace ariles
{
    namespace bridge
    {
        namespace rapidjson
        {
            class Flags : public ariles::Flags<unsigned int, Flags>
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
                class Base : public t_Base
            {
                protected:
                    typedef t_Implementation Impl;
                    typedef ARILES_SHARED_PTR<t_Implementation> ImplPtr;

                protected:
                    ImplPtr impl_;
                    Flags flags_;


                private:
                    Base(const Base&);
                    Base& operator=(const Base&);

                protected:
                    Base(){};
                    ~Base(){};
                    Base(const Flags &flags) : flags_(flags) {};


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


#include "./rapidjson/reader.h"
#include "./rapidjson/writer.h"


#ifndef ARILES_BRIDGE_INCLUDED_rapidjson
    namespace ariles
    {
        /**
         * @brief JSON bridge.
         */
        struct ARILES_VISIBILITY_ATTRIBUTE rapidjson
        {
            typedef ariles::bridge::rapidjson::Flags Flags;

            typedef bridge::rapidjson::Reader ReaderBase;
            typedef bridge::rapidjson::Writer WriterBase;

            typedef ariles::cfgread::Visitor<bridge::rapidjson::Reader> Reader;
            typedef ariles::cfgwrite::Visitor<bridge::rapidjson::Writer> Writer;
        };
    }
#endif
