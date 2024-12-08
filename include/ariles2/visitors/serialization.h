/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2020 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include "common.h"

/**
@defgroup serialization Serialization

@brief Serialization.
*/

namespace ariles2
{
    /// @ingroup serialization
    namespace serialization
    {
        class ARILES2_VISIBILITY_PUBLIC Parameters : public visitor::Parameters
        {
        public:
            bool sloppy_maps_;                /// Treat key values in maps as entry names if they are strings
            bool sloppy_pairs_;               /// Treat first entry in an std::pair as entry name if it is a string
            bool explicit_matrix_size_;       /// Specify matrix size even if it is known to be constant
            bool fallback_to_string_floats_;  /// Allow saving floats as strings if necessary
            bool flat_matrices_;              /// Save matrix as a single vector
            bool allow_missing_entries_;      /// Do not treat missing entries as errors
            bool persistent_structure_;       /// Hint: expect Ariles classes with constant number of entries


        public:
            explicit Parameters(const bool override_parameters = true) : visitor::Parameters(override_parameters)
            {
                sloppy_maps_ = false;
                sloppy_pairs_ = false;
                explicit_matrix_size_ = false;
                fallback_to_string_floats_ = true;
                flat_matrices_ = true;
                allow_missing_entries_ = false;
                persistent_structure_ = false;
            }
        };


        template <class t_Visitor, class t_Implementation>
        class ARILES2_VISIBILITY_PUBLIC PIMPLVisitor : public t_Visitor
        {
        protected:
            using Impl = t_Implementation;
            using ImplPtr = std::shared_ptr<t_Implementation>;

        protected:
            ImplPtr impl_;

        protected:
            PIMPLVisitor() {};
            ~PIMPLVisitor() {};

            template <class... t_Args>
            void makeImplPtr(t_Args &&...args)
            {
                impl_ = std::make_shared<Impl>(std::forward<t_Args>(args)...);
            }
        };


        template <class t_Derived, class t_Parameters>
        using Base = visitor::Base<t_Derived, t_Parameters>;
    }  // namespace serialization
}  // namespace ariles2
