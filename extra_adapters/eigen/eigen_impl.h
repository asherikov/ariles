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
    namespace reader
    {
        /**
         * @brief Read configuration entry (vector)
         *
         * @tparam t_Scalar Eigen template parameter
         * @tparam t_rows   Eigen template parameter
         * @tparam t_flags  Eigen template parameter
         *
         * @param[out] entry     configuration parameter
         */
        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader &reader,
                    Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                    const ariles::ConfigurableParameters & param)
        {
            std::size_t size = reader.startArray();

            if (Eigen::Dynamic == t_rows)
            {
                entry.resize(size);
            }
            else
            {
                ARILES_ASSERT(  (static_cast<int>(size) == t_rows),
                                "Wrong entry size.");
            }

            for(EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
            {
                readBody(reader, entry[i], param);
                reader.shiftArray();
            }
            reader.endArray();
        }



        /**
         * @brief Read a configuration entry (matrix)
         *
         * @tparam t_Scalar Eigen template parameter
         * @tparam t_rows   Eigen template parameter
         * @tparam t_cols   Eigen template parameter
         * @tparam t_flags  Eigen template parameter
         *
         * @param[out] entry      data
         */
        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(  t_Reader & reader,
                       Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                       const ariles::ConfigurableParameters & param)
        {
            EIGEN_DEFAULT_DENSE_INDEX_TYPE num_rows;
            EIGEN_DEFAULT_DENSE_INDEX_TYPE num_cols;

            ariles::ConfigurableParameters param_local = param;
            param_local.crash_on_missing_entry_ = true;

            reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(3);
            readEntry(reader, num_cols, "cols", param_local);
            readEntry(reader, num_rows, "rows", param_local);


            Eigen::VectorXd v;
            readEntry(reader, v, "data", param_local);
            reader.endMap();

            ARILES_ASSERT(v.rows() == num_rows*num_cols, "Wrong entry size.");

            Eigen::Map<
                Eigen::Matrix<  double,
                                Eigen::Dynamic,
                                Eigen::Dynamic,
                                Eigen::RowMajor> >  map(v.data(),
                                                    num_rows,
                                                    num_cols);
            entry = map;
        }


        template <  class t_Reader,
                    typename t_Scalar,
                    int t_dim,
                    int t_mode,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader & reader,
                        Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                        const ariles::ConfigurableParameters & param)
        {
            Eigen::Matrix<
                t_Scalar,
                Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim+1,
                Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim+1> raw_matrix;
            readBody(reader, raw_matrix, param);
            entry.matrix() = raw_matrix;
        }


        template <  class t_Reader,
                    typename t_Scalar,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(   t_Reader & reader,
                        Eigen::Quaternion< t_Scalar, t_options > &entry,
                        const ariles::ConfigurableParameters & param)
        {
            ariles::ConfigurableParameters param_local = param;
            param_local.crash_on_missing_entry_ = true;

            reader.template startMap<t_Reader::SIZE_LIMIT_EQUAL>(4);
            readEntry(reader, entry.x(), "x", param_local);
            readEntry(reader, entry.y(), "y", param_local);
            readEntry(reader, entry.z(), "z", param_local);
            readEntry(reader, entry.w(), "w", param_local);
            reader.endMap();
        }
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
            writeBody( t_Writer & writer,
                       const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                       const ariles::ConfigurableParameters & /*param*/)
        {
            writer.startArray(entry.rows(), true);
            for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < entry.rows(); ++i)
            {
                writer.writeElement(entry[i]);
                writer.shiftArray();
            }
            writer.endArray();
        }



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
            writeBody( t_Writer & writer,
                       const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                       const ariles::ConfigurableParameters & param)
        {
            writer.startMap(3);

            writeEntry(writer, entry.cols(), "cols", param);
            writeEntry(writer, entry.rows(), "rows", param);


            writer.descend("data");
            writer.startArray(entry.size(), true);
            for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < entry.rows(); ++i)
            {
                for (EIGEN_DEFAULT_DENSE_INDEX_TYPE j = 0; j < entry.cols(); ++j)
                {
                    writer.writeElement(entry(i, j));
                    writer.shiftArray();
                }
            }
            writer.endArray();
            writer.ascend();

            writer.endMap();
        }


        template <  class t_Writer,
                    typename t_Scalar,
                    int t_dim,
                    int t_mode,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                        const ariles::ConfigurableParameters & param)
        {
            writeBody(writer, entry.matrix(), param);
        }


        template <  class t_Writer,
                    typename t_Scalar,
                    int t_options>
            void ARILES_VISIBILITY_ATTRIBUTE
            writeBody(  t_Writer & writer,
                        const Eigen::Quaternion< t_Scalar, t_options > &entry,
                        const ariles::ConfigurableParameters & param)
        {
            writer.startMap(4);

            writeEntry(writer, entry.x(), "x", param);
            writeEntry(writer, entry.y(), "y", param);
            writeEntry(writer, entry.z(), "z", param);
            writeEntry(writer, entry.w(), "w", param);

            writer.endMap();
        }
    }
}
