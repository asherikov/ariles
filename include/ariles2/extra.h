/**
    @file
    @author  Alexander Sherikov

    @copyright 2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief Extra ariles types, not required.
*/

#pragma once

#include "ariles.h"


#ifdef ARILES2_ENABLED

namespace ariles2
{
    namespace serialization
    {
        namespace parameters
        {
            template <class t_Parameters>
            class SloppyMixin : public t_Parameters
            {
            public:
                SloppyMixin()
                {
                    this->sloppy_maps_ = true;
                    this->sloppy_pairs_ = true;
                }
            };

            template <class t_Parameters>
            class RelaxedMixin : public t_Parameters
            {
            public:
                RelaxedMixin()
                {
                    this->allow_missing_entries_ = true;
                }
            };

            template <class t_Parameters>
            class NonFlatMatricesMixin : public t_Parameters
            {
            public:
                NonFlatMatricesMixin()
                {
                    this->flat_matrices_ = false;
                }
            };
        }  // namespace parameters


        template <class t_ReadParameters, class t_WriteParameters>
        class NonDefaultBaseTemplate : public DefaultBase
        {
        public:
            using DefaultBase::arilesGetParameters;

            virtual const read::Visitor::Parameters &arilesGetParameters(const read::Visitor &) const
            {
                ARILES2_TRACE_FUNCTION;
                const static t_ReadParameters parameters;
                return (parameters);
            }

            virtual const write::Visitor::Parameters &arilesGetParameters(const write::Visitor &) const
            {
                ARILES2_TRACE_FUNCTION;
                const static t_WriteParameters parameters;
                return (parameters);
            }
        };
    }  // namespace serialization


    namespace read
    {
        using SloppyParameters = serialization::parameters::SloppyMixin<Parameters>;
        using RelaxedParameters = serialization::parameters::RelaxedMixin<Parameters>;
        using NonFlatMatricesParameters = serialization::parameters::NonFlatMatricesMixin<Parameters>;
        using RelaxedSloppyParameters =
                serialization::parameters::RelaxedMixin<serialization::parameters::SloppyMixin<Parameters>>;
        using NonFlatMatricesSloppyParameters =
                serialization::parameters::NonFlatMatricesMixin<serialization::parameters::SloppyMixin<Parameters>>;
        using NonFlatMatricesRelaxedSloppyParameters = serialization::parameters::NonFlatMatricesMixin<
                serialization::parameters::RelaxedMixin<serialization::parameters::SloppyMixin<Parameters>>>;
    }  // namespace read

    namespace write
    {
        using SloppyParameters = serialization::parameters::SloppyMixin<Parameters>;
        using RelaxedParameters = serialization::parameters::RelaxedMixin<Parameters>;
        using NonFlatMatricesParameters = serialization::parameters::NonFlatMatricesMixin<Parameters>;
        using RelaxedSloppyParameters =
                serialization::parameters::RelaxedMixin<serialization::parameters::SloppyMixin<Parameters>>;
        using NonFlatMatricesSloppyParameters =
                serialization::parameters::NonFlatMatricesMixin<serialization::parameters::SloppyMixin<Parameters>>;
        using NonFlatMatricesRelaxedSloppyParameters = serialization::parameters::NonFlatMatricesMixin<
                serialization::parameters::RelaxedMixin<serialization::parameters::SloppyMixin<Parameters>>>;
    }  // namespace write


    class SloppyBase : public serialization::NonDefaultBaseTemplate<read::SloppyParameters, write::SloppyParameters>
    {
    };


    class RelaxedSloppyBase
      : public serialization::NonDefaultBaseTemplate<read::RelaxedSloppyParameters, write::RelaxedSloppyParameters>
    {
    };

    class NonFlatMatricesRelaxedSloppyBase : public serialization::NonDefaultBaseTemplate<
                                                     read::NonFlatMatricesRelaxedSloppyParameters,
                                                     write::NonFlatMatricesRelaxedSloppyParameters>
    {
    };
}  // namespace ariles2

#else

namespace ariles2
{
    // Some classes may inherit from this
    using SloppyBase = DefaultBase;
    using RelaxedSloppyBase = DefaultBase;
    using NonFlatMatricesRelaxedSloppyBase = DefaultBase;
}  // namespace ariles2

#endif
