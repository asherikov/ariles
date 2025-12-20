/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2024 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/
// cppcheck-suppress-file duplInheritedMember

#pragma once

#include "common.h"

namespace ariles2
{
    /// @ingroup aggregate
    namespace aggregate
    {
        template <class t_Visitor>
        class ParametersWrapper
        {
        public:
            typename t_Visitor::Parameters parameters_;

        public:
            explicit ParametersWrapper(const bool override_parameters = true) : parameters_(override_parameters)
            {
            }

            explicit ParametersWrapper(const typename t_Visitor::Parameters &parameters) : parameters_(parameters)
            {
            }

            template <
                    class t_IdType,
                    typename = std::enable_if_t<
                            std::is_same<t_IdType, t_Visitor>::value
                            or std::is_base_of<t_IdType, typename t_Visitor::Parameters>::value>>
            typename t_Visitor::Parameters &get()
            {
                return (parameters_);
            }

            template <
                    class t_IdType,
                    typename = std::enable_if_t<
                            std::is_same<t_IdType, t_Visitor>::value
                            or std::is_base_of<t_IdType, typename t_Visitor::Parameters>::value>>
            const typename t_Visitor::Parameters &get() const
            {
                return (parameters_);
            }
        };

        template <class... t_Visitors>
        class Parameters;

        template <>
        class Parameters<>
        {
        public:
            explicit Parameters(const bool){};
            Parameters(){};

            void get(){};
        };

        template <class t_Visitor, class... t_Visitors>
        class Parameters<t_Visitor, t_Visitors...> : public ParametersWrapper<t_Visitor>,
                                                     public Parameters<t_Visitors...>
        {
        public:
            explicit Parameters(const bool override_parameters = true)
              : ParametersWrapper<t_Visitor>(override_parameters), Parameters<t_Visitors...>(override_parameters)
            {
            }

            template <class... t_Parameters>
            // cppcheck-suppress noExplicitConstructor
            Parameters(const typename t_Visitor::Parameters &parameters, t_Parameters &&...other_parameters)
              : ParametersWrapper<t_Visitor>(parameters)
              , Parameters<t_Visitors...>(std::forward<t_Parameters>(other_parameters)...)
            {
            }

            template <class t_Parameters>
            // cppcheck-suppress noExplicitConstructor
            Parameters(const t_Parameters &parameters, const bool override_parameters = true)
              : ParametersWrapper<t_Visitor>(override_parameters)
              , Parameters<t_Visitors...>(parameters, override_parameters)
            {
            }

            // cppcheck-suppress noExplicitConstructor
            Parameters(const typename t_Visitor::Parameters &parameters, const bool override_parameters = true)
              : ParametersWrapper<t_Visitor>(parameters), Parameters<t_Visitors...>(override_parameters)
            {
            }

            using ParametersWrapper<t_Visitor>::get;
            using Parameters<t_Visitors...>::get;
        };



        template <class t_Visitor>
        class BaseVisitorWrapper
        {
        public:
            t_Visitor visitor_;

        public:
            template <class... t_Args>
            explicit BaseVisitorWrapper(const std::tuple<t_Args...> &args_tuple)
              : visitor_(std::get<t_Args>(args_tuple)...)
            {
            }

            template <
                    class t_GetVisitor,
                    typename = std::enable_if_t<
                            std::is_same<t_GetVisitor, t_Visitor>::value
                            or std::is_base_of<t_GetVisitor, t_Visitor>::value>>
            t_Visitor &get()
            {
                return (visitor_);
            }

            template <
                    class t_GetVisitor,
                    typename = std::enable_if_t<
                            std::is_same<t_GetVisitor, t_Visitor>::value
                            or std::is_base_of<t_GetVisitor, t_Visitor>::value>>
            const t_Visitor &get() const
            {
                return (visitor_);
            }
        };

        template <class... t_Visitors>
        class BaseVisitor;

        template <>
        class BaseVisitor<>
        {
        public:
            BaseVisitor()
            {
            }

            void get() const
            {
            }

            template <class t_Entry, class t_Path, class t_Parameters>
            void visit(t_Entry &, const t_Path &, const t_Parameters &)
            {
            }
        };

        template <class t_Visitor, class... t_Visitors>
        class BaseVisitor<t_Visitor, t_Visitors...> : public BaseVisitorWrapper<t_Visitor>,
                                                      public BaseVisitor<t_Visitors...>
        {
        public:
            template <class t_ConstructorTuple, class... t_Args>
            BaseVisitor(t_ConstructorTuple &&constructor_tuple, t_Args &&...args)
              : BaseVisitorWrapper<t_Visitor>(std::forward<t_ConstructorTuple>(constructor_tuple))
              , BaseVisitor<t_Visitors...>(std::forward<t_Args>(args)...)
            {
            }

            using BaseVisitorWrapper<t_Visitor>::get;
            using BaseVisitor<t_Visitors...>::get;


            template <class t_Entry, class t_Path, class t_Parameters>
            void visit(t_Entry &entry, const t_Path &path, const t_Parameters &param)
            {
                ariles2::apply(BaseVisitorWrapper<t_Visitor>::visitor_, entry, path, param.template get<t_Visitor>());

                BaseVisitor<t_Visitors...>::visit(entry, path, param);
            }
        };


        template <class t_Derived, class... t_Visitors>
        class Visitor : public BaseVisitor<t_Visitors...>, public visitor::Base<t_Derived, Parameters<t_Visitors...>>
        {
        public:
            using Parameters = aggregate::Parameters<t_Visitors...>;

        public:
            template <class... t_Args>
            explicit Visitor(t_Args &&...args) : BaseVisitor<t_Visitors...>(std::forward<t_Args>(args)...)
            {
            }

            template <class t_Ariles>
            const Parameters getParameters(const t_Ariles &ariles_class) const
            {
                // static variable is potentially unsafe
                return (Parameters(
                        ariles_class.arilesGetParameters(BaseVisitor<t_Visitors...>::template get<t_Visitors>())...));
            }


            template <class t_Entry, class t_Path>
            void visit(t_Entry &entry, const t_Path &path, const Parameters &param)
            {
                BaseVisitor<t_Visitors...>::visit(entry, path, param);
            }
        };
    }  // namespace aggregate
}  // namespace ariles2
