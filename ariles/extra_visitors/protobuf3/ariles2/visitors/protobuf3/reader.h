/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

namespace ariles2
{
    namespace ns_protobuf3
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Reader : public visitor::Base<visitor::Visitor, visitor::Parameters>
        {
        public:
            typedef visitor::Parameters Parameters;


        public:
            using visitor::Base<visitor::Visitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            template <class t_Left, class t_Right>
            void visit(t_Left &left, const t_Right &right, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                try
                {
                    this->visitMapEntry(left, right, name, param);
                }
                catch (std::exception &e)
                {
                    ARILES2_THROW(std::string("Copying failed: ") + e.what());
                }
            }


            template <class t_Left, class t_Right>
            void visitMapEntry(t_Left &left, const t_Right &right, const std::string &name, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_VALUE(name);
                ARILES2_TRACE_TYPE(left);
                ARILES2_TRACE_TYPE(right);

                try
                {
                    copyfrom::apply_copyfrom(*this, left, right, param);
                }
                catch (const std::exception &e)
                {
                    ARILES2_THROW("entry: " + name + " // " + std::string(e.what()));
                }
            }
        };
    }  // namespace ns_protobuf3
}  // namespace ariles2
