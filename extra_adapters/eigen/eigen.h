/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#define ARILES_INCLUDED_ADAPTER_EIGEN

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace ariles
{
    namespace adapter
    {
        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader &reader,
                        Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                        const ariles::ConfigurableFlags & param);


        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader & reader,
                        Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                        const ariles::ConfigurableFlags & param);


        template <  class t_Reader,
                    typename t_Scalar,
                    int t_dim,
                    int t_mode,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader & reader,
                        Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                        const ariles::ConfigurableFlags & param);


        template <  class t_Reader,
                    typename t_Scalar,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader & reader,
                        Eigen::Quaternion< t_Scalar, t_options > &entry,
                        const ariles::ConfigurableFlags & param);


        // ====================================================


        template <  class t_Writer,
                    typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                        const ariles::ConfigurableFlags & param);



        template <  class t_Writer,
                    typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                        const ariles::ConfigurableFlags & param);


        template <  class t_Writer,
                    typename t_Scalar,
                    int t_dim,
                    int t_mode,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                        const ariles::ConfigurableFlags & param);


        template <  class t_Writer,
                    typename t_Scalar,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Quaternion< t_Scalar, t_options > &entry,
                        const ariles::ConfigurableFlags & param);


        // ====================================================


        template <  typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            setDefaults(Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry);


        template <  typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            setDefaults(Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry);


        template <  typename t_Scalar,
                    int t_dim,
                    int t_mode,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            setDefaults(Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry);


        template <  typename t_Scalar,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            setDefaults(Eigen::Quaternion< t_Scalar, t_options > &entry);
    }
}
