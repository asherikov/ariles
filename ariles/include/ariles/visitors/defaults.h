/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <limits>
#include "common.h"

namespace ariles
{
    namespace defaults
    {
        class Parameters
        {
            public:
                double default_double_value_;
                float default_float_value_;

            public:
                Parameters()
                {
                    default_double_value_ = ARILES_DEFAULT_DOUBLE_VALUE;
                    default_float_value_ = ARILES_DEFAULT_FLOAT_VALUE;
                }

                template<typename t_Scalar>
                inline t_Scalar getDefault() const
                {
                    return 0;
                }
        };


        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::VisitorBase<defaults::Parameters>
        {
            public:
                typedef defaults::Parameters Parameters;


            public:
                using visitor::VisitorBase<Parameters>::getDefaultParameters;

                template<class t_Ariles>
                    const Parameters & getParameters(const t_Ariles & ariles_class) const
                {
                    return (ariles_class.arilesGetParameters(*this));
                }

                using visitor::VisitorBase<Parameters>::startRoot;
                using visitor::VisitorBase<Parameters>::finishRoot;


                template<class t_Entry>
                    void operator()(
                            t_Entry & entry,
                            const std::string & name,
                            const Parameters & param) const
                {
                    ARILES_UNUSED_ARG(name);
                    ARILES_TRACE_FUNCTION;
                    ARILES_TRACE_ENTRY(name);
                    ARILES_TRACE_TYPE(entry);
                    apply_defaults(*this, entry, param);
                }
        };

        template<>
        inline double Visitor::Parameters::getDefault<double>() const
        {
            return default_double_value_;
        }

        template<>
        inline float Visitor::Parameters::getDefault<float>() const
        {
            return default_float_value_;
        }

        template<>
        inline bool Visitor::Parameters::getDefault<bool>() const
        {
            return false;
        }

        template<>
        inline std::string Visitor::Parameters::getDefault<std::string>() const
        {
            return "";
        }


        class ARILES_VISIBILITY_ATTRIBUTE Base
            : public visitor::Base<const defaults::Visitor>
        {
            public:
        };


#ifndef ARILES_METHODS_defaults
#   define ARILES_METHODS_defaults ARILES_METHODS(defaults, const, ARILES_EMPTY_MACRO)
#endif
    }


    typedef defaults::Visitor Defaults;
}
