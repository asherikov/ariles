/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once


namespace ariles
{
    namespace visitor
    {
        struct ARILES_VISIBILITY_ATTRIBUTE Visitor
        {
            public:
                typedef int VisitorIndicatorType;
        };


        template<class t_Derived, class t_Visitor, class t_Parameters>
            class ARILES_VISIBILITY_ATTRIBUTE Base
        {
            public:
                virtual void arilesApply(t_Visitor &, t_Parameters &) = 0;
                virtual void ariles(t_Visitor &visitor,
                                    const std::string & name,
                                    t_Parameters &param) = 0;

                void ariles(t_Visitor &visitor,
                            t_Parameters &param)
                {
                    ARILES_TRACE_FUNCTION;
                    ariles(visitor, static_cast<t_Derived*>(this)->arilesDefaultID(), param);
                }

                void ariles(t_Visitor &visitor)
                {
                    ARILES_TRACE_FUNCTION;
                    ariles(visitor, visitor.default_parameters_);
                }
        };


        template<class t_Derived, class t_Visitor, class t_Parameters>
            class ARILES_VISIBILITY_ATTRIBUTE ConstBase
        {
            public:
                virtual void arilesApply(t_Visitor &, t_Parameters &) const = 0;
                virtual void ariles(t_Visitor &visitor,
                                    const std::string & name,
                                    t_Parameters &param) const = 0;

                void ariles(t_Visitor &visitor,
                            t_Parameters &param) const
                {
                    ARILES_TRACE_FUNCTION;
                    ariles(visitor, static_cast<t_Derived*>(this)->arilesDefaultID(), param);
                }

                void ariles(t_Visitor &visitor) const
                {
                    ARILES_TRACE_FUNCTION;
                    ariles(visitor, visitor.default_parameters_);
                }
        };
    }
}
