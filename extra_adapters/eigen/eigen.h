/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_ADAPTER_EIGEN

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ariles
{
    namespace reader
    {
        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader &reader,
                        Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                        const bool crash_on_missing_entry = false);

        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader & reader,
                        Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                        const bool crash_on_missing_entry = false);
    }


    namespace writer
    {
        /**
         * @brief Write a configuration entry (vector)
         *
         * @tparam t_Derived Eigen template parameter
         *
         * @param[in] entry      data
         * @param[in] entry_name name
         */
        template <  class t_Writer,
                    typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry);



        /**
         * @brief Write a configuration entry (matrix)
         *
         * @tparam t_Scalar Eigen template parameter
         * @tparam t_rows   Eigen template parameter
         * @tparam t_cols   Eigen template parameter
         * @tparam t_flags  Eigen template parameter
         *
         * @param[in] entry      data
         * @param[in]  entry_name name
         */
        template <  class t_Writer,
                    typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry);
    }
}
