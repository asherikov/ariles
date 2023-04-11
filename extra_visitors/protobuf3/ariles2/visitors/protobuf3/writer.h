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
        class ARILES2_VISIBILITY_ATTRIBUTE Writer : public visitor::Base<visitor::Visitor, visitor::Parameters>
        {
        public:
            using Parameters = visitor::Parameters;


        public:
            using visitor::Base<visitor::Visitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }


            template <class t_Left, class t_Right>
            void visit(const t_Left &left, t_Right &right, const std::string & /*name*/, const Parameters &param)
            {
                ARILES2_TRACE_FUNCTION;
                try
                {
                    left.arilesVisit(*this, right, param);
                }
                catch (std::exception &e)
                {
                    ARILES2_THROW(std::string("Copying failed: ") + e.what());
                }
            }
        };
    }  // namespace ns_protobuf3
}  // namespace ariles2
