/**
    @file
    @author Alexander Sherikov

    @copyright 2018-2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

/**
@defgroup namevalue2 NameValue2
@ingroup config

@brief Generates a set of <std::string, double> pairs with flattened member names,
e.g., <"ariles_class.class_member.real_member", 3.4>.
*/


#pragma once

#define ARILES2_VISITOR_INCLUDED_namevalue2

#include <ariles2/internal/helpers.h>
#include <ariles2/visitors/config.h>

#include <string>
#include <memory>


namespace ariles2
{
    namespace ns_namevalue2
    {
        namespace impl
        {
            class ARILES2_VISIBILITY_PUBLIC Writer;
        }

        class ARILES2_VISIBILITY_PUBLIC NameValueContainer
        {
        public:
            virtual ~NameValueContainer() = default;

            virtual std::string &name(const std::size_t index) = 0;
            virtual double &value(const std::size_t index) = 0;
            virtual std::size_t size() const = 0;
            virtual void reserve(const std::size_t size) = 0;
            virtual void resize(const std::size_t size) = 0;
        };


        /**
         * @brief Configuration writer class
         */
        class ARILES2_VISIBILITY_PUBLIC Writer : public serialization::PIMPLVisitor<write::Visitor, impl::Writer>
        {
        public:
            explicit Writer(const std::shared_ptr<NameValueContainer> &container, const std::size_t reserve = 0);
            virtual void startRoot(const std::string &name, const Parameters &param);
            void flush();
            virtual void startMap(const Parameters &, const std::size_t num_entries);
            virtual void startMapEntry(const std::string &map_name);
            virtual void endMapEntry();
            virtual void endMap();
            virtual bool startIteratedMap(const std::size_t /*num_entries*/, const Parameters &);
            virtual void startArray(const std::size_t size, const bool /*compact*/ = false);
            virtual void endArrayElement();
            virtual void endArray();


#define ARILES2_BASIC_TYPE(type) void writeElement(const type &element, const Parameters &);

            CPPUT_MACRO_SUBSTITUTE(ARILES2_BASIC_NUMERIC_TYPES_LIST)

#undef ARILES2_BASIC_TYPE


            void writeElement(const std::string &element, const Parameters &parameters);
            virtual const Parameters &getDefaultParameters() const;
        };
    }  // namespace ns_namevalue2
}  // namespace ariles2


namespace ariles2
{
    /**
     * @brief NameValue2 visitor.
     * @ingroup namevalue2
     */
    struct ARILES2_VISIBILITY_PUBLIC namevalue2
    {
        using NameValueContainer = ns_namevalue2::NameValueContainer;
        using Writer = ns_namevalue2::Writer;
    };
}  // namespace ariles2
