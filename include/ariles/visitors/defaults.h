/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

namespace ariles
{
    namespace defaults
    {
        class ARILES_VISIBILITY_ATTRIBUTE Visitor : public ariles::visitor::Visitor
        {
            public:
                class DefaultsParameters
                {
                    public:
                        double default_double_value_;
                        float default_float_value_;

                    public:
                        DefaultsParameters()
                        {
                            default_double_value_ = ARILES_DEFAULT_DOUBLE_VALUE;
                            default_float_value_ = ARILES_DEFAULT_FLOAT_VALUE;
                        }

                        template<typename t_Scalar>
                        inline t_Scalar getDefault() const
                        {
                            return 0;
                        }
                } default_parameters_;


            public:
                template<class t_Configurable>
                    void startRoot( const t_Configurable &,
                                    const DefaultsParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void finishRoot(const t_Configurable &,
                                    const DefaultsParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }
        };

        template<>
        inline double Visitor::DefaultsParameters::getDefault<double>() const
        {
            return default_double_value_;
        }

        template<>
        inline float Visitor::DefaultsParameters::getDefault<float>() const
        {
            return default_float_value_;
        }

        template<>
        inline bool Visitor::DefaultsParameters::getDefault<bool>() const
        {
            return false;
        }

        template<>
        inline std::string Visitor::DefaultsParameters::getDefault<std::string>() const
        {
            return "";
        }


        template<class t_Derived>
            class ARILES_VISIBILITY_ATTRIBUTE Base
                : public visitor::Base<t_Derived, const defaults::Visitor, const defaults::Visitor::DefaultsParameters>
        {
            public:
        };
    }
}
