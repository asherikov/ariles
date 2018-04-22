/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_BOOSTPTR

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader &reader,
                        boost::shared_ptr<t_Entry> &entry,
                        const bool crash_on_missing_entry = false);
    }


    namespace writer
    {
        template <  class t_Writer,
                    typename t_Entry>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const boost::shared_ptr<t_Entry> &entry);
    }
}
