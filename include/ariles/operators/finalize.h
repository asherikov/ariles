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
    namespace finalize
    {
        class ARILES_VISIBILITY_ATTRIBUTE Iterator
        {
            public:
                class FinalizeParameters
                {
                } default_parameters_;


            public:
                template<class t_Configurable>
                    void startBody( const t_Configurable &,
                                    const FinalizeParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void finishBody(t_Configurable & configurable,
                                    const FinalizeParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                    configurable.finalize();
                }


                template<class t_Configurable>
                    void startRoot( const t_Configurable &,
                                    const FinalizeParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                }


                template<class t_Configurable>
                    void finishRoot(t_Configurable & configurable,
                                    const FinalizeParameters &) const
                {
                    ARILES_TRACE_FUNCTION;
                    configurable.finalize();
                }
        };


        class ARILES_VISIBILITY_ATTRIBUTE Base
        {
            public:
                virtual void arilesApply(   const ariles::finalize::Iterator &,
                                            const ariles::finalize::Iterator::FinalizeParameters &) = 0;


                /**
                 * @todo DEPRECATED
                 * @{
                 */
                virtual void arilesFinalize()
                {
                    ariles::finalize::Iterator iterator;
                    arilesApply(iterator, iterator.default_parameters_);
                }

                virtual void finalize()
                {
                }
                /// @}
        };
    }
}
