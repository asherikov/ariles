/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <limits>

namespace ariles
{
    namespace defaults
    {
        class ARILES_VISIBILITY_ATTRIBUTE Iterator
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
                void start(t_Configurable & /*configurable*/, DefaultsParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                void finish(t_Configurable & /*configurable*/, DefaultsParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                }
        };

        template<>
        inline double Iterator::DefaultsParameters::getDefault<double>() const
        {
            return default_double_value_;
        }

        template<>
        inline float Iterator::DefaultsParameters::getDefault<float>() const
        {
            return default_float_value_;
        }

        template<>
        inline bool Iterator::DefaultsParameters::getDefault<bool>() const
        {
            return false;
        }

        template<>
        inline std::string Iterator::DefaultsParameters::getDefault<std::string>() const
        {
            return "";
        }


        class Base
        {
            public:
                /**
                 * @brief Set members to their default values.
                 */
                virtual void arilesApply(   ariles::defaults::Iterator &,
                                            const ariles::defaults::Iterator::DefaultsParameters &) = 0;
                virtual void arilesApply(   ariles::defaults::Iterator & iterator)
                {
                    ARILES_TRACE_FUNCTION;
                    arilesApply(iterator, iterator.default_parameters_);
                }

                /// @todo DEPRECATED
                void setDefaults()
                {
                    ariles::defaults::Iterator iterator;
                    arilesApply(iterator);
                }
        };
    }
}
