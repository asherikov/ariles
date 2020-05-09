/**
    @file
    @author  Alexander Sherikov

    @copyright 2017-2018 Alexander Sherikov, Licensed under the Apache License, Version 2.0.
    (see @ref LICENSE or http://www.apache.org/licenses/LICENSE-2.0)

    @brief
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "../visitors/serialization.h"
#include "../internal/helpers.h"

namespace ariles
{
    namespace read
    {
        template <class t_Visitor, typename t_Scalar, int t_rows, int t_flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            std::size_t size = visitor.startArray();

            if (Eigen::Dynamic == t_rows)
            {
                entry.resize(size);
            }
            else
            {
                ARILES_ASSERT((static_cast<int>(size) == t_rows), "Wrong entry size.");
            }

            for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
            {
                apply_read(visitor, entry[i], param);
                visitor.shiftArray();
            }
            visitor.endArray();
        }



        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES_TRACE_FUNCTION;
            if (Eigen::Dynamic == t_rows || Eigen::Dynamic == t_cols
                || parameters.isSet(ConfigurableFlags::FORCE_EXPLICIT_MATRIX_SIZE))
            {
                EIGEN_DEFAULT_DENSE_INDEX_TYPE num_rows;
                EIGEN_DEFAULT_DENSE_INDEX_TYPE num_cols;


                ariles::ConfigurableFlags param = parameters;
                param.set(ConfigurableFlags::DISABLE_ALLOW_MISSING_ENTRIES);

                visitor.template startMap<t_Visitor::SIZE_LIMIT_EQUAL>(3);
                visitor(num_cols, "cols", param);
                ARILES_ASSERT(Eigen::Dynamic == t_cols || t_cols == num_cols, "Wrong number of columns.");
                visitor(num_rows, "rows", param);
                ARILES_ASSERT(Eigen::Dynamic == t_rows || t_rows == num_rows, "Wrong number of rows.");


                Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1> v;
                visitor(v, "data", param);
                visitor.endMap();

                ARILES_ASSERT(v.rows() == num_rows * num_cols, "Wrong entry size.");

                Eigen::Map<Eigen::Matrix<t_Scalar, t_rows, t_cols, Eigen::RowMajor> > map(v.data(), num_rows, num_cols);
                entry = map;
            }
            else
            {
                Eigen::Matrix<t_Scalar, t_rows * t_cols, 1> v;

                apply_read(visitor, v, parameters);

                Eigen::Map<Eigen::Matrix<double, t_rows, t_cols, Eigen::RowMajor> > map(v.data(), t_rows, t_cols);
                entry = map;
            }
        }


        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            Eigen::Matrix<
                    t_Scalar,
                    Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim + 1,
                    Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim + 1>
                    raw_matrix;
            apply_read(visitor, raw_matrix, param);
            entry.matrix() = raw_matrix;
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Quaternion<t_Scalar, t_options> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES_TRACE_FUNCTION;

            ariles::ConfigurableFlags param = parameters;
            param.set(ConfigurableFlags::DISABLE_ALLOW_MISSING_ENTRIES);

            visitor.template startMap<t_Visitor::SIZE_LIMIT_EQUAL>(4);
            visitor(entry.x(), "x", param);
            visitor(entry.y(), "y", param);
            visitor(entry.z(), "z", param);
            visitor(entry.w(), "w", param);
            visitor.endMap();
        }
    }  // namespace read
}  // namespace ariles


namespace ariles
{
    namespace write
    {
        template <class t_Visitor, typename t_Scalar, int t_rows, int t_flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            if (writer.getSerializationFeatures().isSet(serialization::Features::NATIVE_MATRIX_SUPPORTED))
            {
                writer.startMatrix(true);
                for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
                {
                    writer.startMatrixRow();
                    writer.writeElement(entry(i));
                    writer.endMatrixRow();
                }
                writer.endMatrix();
            }
            else
            {
                writer.startArray(entry.rows(), true);
                for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < entry.rows(); ++i)
                {
                    writer.writeElement(entry[i]);
                    writer.shiftArray();
                }
                writer.endArray();
            }
        }



        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            if (writer.getSerializationFeatures().isSet(serialization::Features::NATIVE_MATRIX_SUPPORTED))
            {
                writer.startMatrix();
                for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
                {
                    writer.startMatrixRow();
                    for (EIGEN_DEFAULT_DENSE_INDEX_TYPE j = 0; j < (Eigen::Dynamic == t_cols ? entry.cols() : t_cols);
                         ++j)
                    {
                        writer.writeElement(entry(i, j));
                    }
                    writer.endMatrixRow();
                }
                writer.endMatrix();
            }
            else
            {
                if (Eigen::Dynamic == t_rows || Eigen::Dynamic == t_cols
                    || param.isSet(ConfigurableFlags::FORCE_EXPLICIT_MATRIX_SIZE))
                {
                    writer.startMap(3);

                    writer(entry.cols(), "cols", param);
                    writer(entry.rows(), "rows", param);


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
                else
                {
                    writer.startArray(entry.size(), true);
                    for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < t_rows; ++i)
                    {
                        for (EIGEN_DEFAULT_DENSE_INDEX_TYPE j = 0; j < t_cols; ++j)
                        {
                            writer.writeElement(entry(i, j));
                            writer.shiftArray();
                        }
                    }
                    writer.endArray();
                }
            }
        }


        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            apply_write(writer, entry.matrix(), param);
        }


        template <class t_Visitor, typename t_Scalar, int t_options, class t_Flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Quaternion<t_Scalar, t_options> &entry,
                const t_Flags &param)
        {
            ARILES_TRACE_FUNCTION;

            writer.startMap(4);

            writer(entry.x(), "x", param);
            writer(entry.y(), "y", param);
            writer(entry.z(), "z", param);
            writer(entry.w(), "w", param);

            writer.endMap();
        }
    }  // namespace write
}  // namespace ariles



namespace ariles
{
    namespace compare
    {
        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &left,
                const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            visitor.equal_ &= (left.isApprox(right, param.template getTolerance<t_Scalar>()));
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const Eigen::Quaternion<t_Scalar, t_options> &left,
                const Eigen::Quaternion<t_Scalar, t_options> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            visitor.equal_ &= (left.isApprox(right, param.template getTolerance<t_Scalar>()));
        }


        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &left,
                const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            visitor.equal_ &= (left.isApprox(right, param.template getTolerance<t_Scalar>()));
        }
    }  // namespace compare
}  // namespace ariles



namespace ariles
{
    namespace defaults
    {
        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES_TRACE_FUNCTION;
            if (Eigen::Dynamic == t_rows)
            {
                if (Eigen::Dynamic == t_cols)
                {
                    entry.resize(0, 0);
                }
                else
                {
                    entry.resize(0, t_cols);
                }
            }
            else
            {
                if (Eigen::Dynamic == t_cols)
                {
                    entry.resize(t_rows, 0);
                }
                else
                {
                    entry.setConstant(param.template getDefault<t_Scalar>());
                }
            }
        }


        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            entry.setIdentity();
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                Eigen::Quaternion<t_Scalar, t_options> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES_TRACE_FUNCTION;
            entry.setIdentity();
        }
    }  // namespace defaults
}  // namespace ariles
