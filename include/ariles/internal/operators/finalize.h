/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
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
                void start(t_Configurable & /*configurable*/, FinalizeParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                }

                template<class t_Configurable>
                void finish(t_Configurable & configurable, FinalizeParameters & /*param*/)
                {
                    ARILES_TRACE_FUNCTION;
                    configurable.finalize();
                }
        };


        class Base
        {
            public:
                virtual void arilesFinalize(ariles::finalize::Iterator &, const ariles::finalize::Iterator::FinalizeParameters &) = 0;
                virtual void arilesFinalize()
                {
                    ARILES_TRACE_FUNCTION;

                    ariles::finalize::Iterator iterator;
                    arilesFinalize(iterator, iterator.default_parameters_);
                }


                /**
                 * @brief This function is called automaticaly after reading
                 * a configuration file.
                 */
                virtual void finalize()
                {
                }
        };
    }
}
