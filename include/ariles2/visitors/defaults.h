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

namespace ariles2
{
    namespace defaults
    {
        class ARILES2_VISIBILITY_ATTRIBUTE Parameters
        {
        public:
            double default_double_value_;
            float default_float_value_;

        public:
            Parameters()
            {
#ifdef ARILES2_DEFAULT_DOUBLE_VALUE
                default_double_value_ = ARILES2_DEFAULT_DOUBLE_VALUE;
#else
                default_double_value_ = std::numeric_limits<double>::signaling_NaN();
#endif

#ifdef ARILES2_DEFAULT_FLOAT_VALUE
                default_float_value_ = ARILES2_DEFAULT_FLOAT_VALUE;
#else
                default_float_value_ = std::numeric_limits<float>::signaling_NaN();
#endif
            }

            template <typename t_Scalar>
            inline t_Scalar getDefault() const
            {
                return 0;
            }
        };


        class ARILES2_VISIBILITY_ATTRIBUTE Visitor
          : public ariles2::visitor::Base<visitor::GenericVisitor, defaults::Parameters>
        {
        public:
            typedef defaults::Parameters Parameters;


        public:
            using visitor::Base<visitor::GenericVisitor, Parameters>::getDefaultParameters;

            template <class t_Ariles>
            const Parameters &getParameters(const t_Ariles &ariles_class) const
            {
                return (ariles_class.arilesGetParameters(*this));
            }

            template <class t_Entry>
            void start(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                ARILES2_TRACE_FUNCTION;
                this->operator()(entry, name, param);
            }


            template <class t_Entry>
            void operator()(t_Entry &entry, const std::string &name, const Parameters &param) const
            {
                ARILES2_UNUSED_ARG(name);
                ARILES2_TRACE_FUNCTION;
                ARILES2_TRACE_ENTRY(name);
                ARILES2_TRACE_TYPE(entry);
                apply_defaults(*this, entry, param);
            }
        };

        template <>
        inline double Visitor::Parameters::getDefault<double>() const
        {
            return default_double_value_;
        }

        template <>
        inline float Visitor::Parameters::getDefault<float>() const
        {
            return default_float_value_;
        }

        template <>
        inline bool Visitor::Parameters::getDefault<bool>() const
        {
            return false;
        }

        template <>
        inline std::string Visitor::Parameters::getDefault<std::string>() const
        {
            return "";
        }


        class ARILES2_VISIBILITY_ATTRIBUTE Base : public entry::Base<const defaults::Visitor>
        {
        public:
        };


#define ARILES2_VISIT_defaults
#define ARILES2_METHODS_defaults ARILES2_METHODS(defaults, const, ARILES2_EMPTY_MACRO)
#define ARILES2_BASE_METHODS_defaults ARILES2_BASE_METHODS(defaults)
    }  // namespace defaults


    typedef defaults::Visitor Defaults;
}  // namespace ariles2
