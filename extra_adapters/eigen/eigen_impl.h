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
         * @param[in] crash_on_missing_entry
         */
        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE readBody(
                    t_Reader &reader,
                    Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                    const bool crash_on_missing_entry)
        {
            ARILES_IGNORE_UNUSED(crash_on_missing_entry);

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
                readBody(reader,entry[i]);
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
         * @param[in] crash_on_missing_entry
         */
        template <  class t_Reader,
                    typename t_Scalar,
                    int t_rows,
                    int t_cols,
                    int t_flags>
            void ARILES_VISIBILITY_ATTRIBUTE
            readBody(  t_Reader & reader,
                       Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                       const bool crash_on_missing_entry)
        {
            ARILES_IGNORE_UNUSED(crash_on_missing_entry);

            EIGEN_DEFAULT_DENSE_INDEX_TYPE num_rows;
            EIGEN_DEFAULT_DENSE_INDEX_TYPE num_cols;

            readEntry(reader, num_rows, "rows", true);
            readEntry(reader, num_cols, "cols", true);


            Eigen::VectorXd v;
            readEntry(reader, v, "data", true);

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
                        const bool crash_on_missing_entry)
        {
            Eigen::Matrix<
                t_Scalar,
                Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim+1,
                Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim+1> raw_matrix;
            readBody(reader, raw_matrix, crash_on_missing_entry);
            entry.matrix() = raw_matrix;
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
                       const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry)
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
                       const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry)
        {
            writer.startMap(3);

            writeEntry(writer, entry.cols(), "cols");
            writeEntry(writer, entry.rows(), "rows");


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
                        const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry)
        {
            writeBody(writer, entry.matrix());
        }
    }
}
