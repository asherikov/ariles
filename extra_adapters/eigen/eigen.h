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

namespace ariles2
{
    namespace read
    {
        template <class t_Visitor, typename t_Scalar, int t_rows, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            std::size_t size = visitor.startVector();

            if (Eigen::Dynamic == t_rows)
            {
                entry.resize(size);
            }
            else
            {
                ARILES2_ASSERT((static_cast<int>(size) == t_rows), "Wrong entry size.");
            }

            for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
            {
                visitor.visitVectorElement(entry[i], param);
            }
            visitor.endVector();
        }



        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;
            const bool dynamic = Eigen::Dynamic == t_rows or Eigen::Dynamic == t_cols;
            std::size_t num_rows = Eigen::Dynamic == t_rows ? 0 : static_cast<std::size_t>(t_cols);
            std::size_t num_cols = Eigen::Dynamic == t_cols ? 0 : static_cast<std::size_t>(t_rows);

            visitor.startMatrix(num_cols, num_rows, dynamic, parameters);

            ARILES2_ASSERT(
                    Eigen::Dynamic == t_cols || static_cast<std::size_t>(t_cols) == num_cols,
                    "Wrong number of columns.");
            ARILES2_ASSERT(
                    Eigen::Dynamic == t_rows || static_cast<std::size_t>(t_rows) == num_rows, "Wrong number of rows.");

            Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1> v;
            apply_read(visitor, v, parameters);

            ARILES2_ASSERT(static_cast<std::size_t>(v.rows()) == num_rows * num_cols, "Wrong entry size.");
            Eigen::Map<Eigen::Matrix<t_Scalar, t_rows, t_cols, Eigen::RowMajor> > map(v.data(), num_rows, num_cols);
            entry = map;

            visitor.endMatrix(dynamic);
        }


        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            Eigen::Matrix<
                    t_Scalar,
                    Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim + 1,
                    Eigen::Dynamic == t_dim ? Eigen::Dynamic : t_dim + 1>
                    raw_matrix;
            apply_read(visitor, raw_matrix, param);
            entry.matrix() = raw_matrix;
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_read(
                t_Visitor &visitor,
                Eigen::Quaternion<t_Scalar, t_options> &entry,
                const typename t_Visitor::Parameters &parameters)
        {
            ARILES2_TRACE_FUNCTION;

            typename t_Visitor::Parameters param = parameters;
            param.allow_missing_entries_ = false;

            visitor.startMap(t_Visitor::SIZE_LIMIT_EQUAL, 4);
            visitor.visitMapEntry(entry.x(), "x", param, true);
            visitor.visitMapEntry(entry.y(), "y", param, true);
            visitor.visitMapEntry(entry.z(), "z", param, true);
            visitor.visitMapEntry(entry.w(), "w", param, true);
            visitor.endMap();
        }
    }  // namespace read
}  // namespace ariles2


namespace ariles2
{
    namespace write
    {
        template <class t_Visitor, typename t_Scalar, int t_rows, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Matrix<t_Scalar, t_rows, 1, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            writer.startVector(entry.rows());
            for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < (Eigen::Dynamic == t_rows ? entry.rows() : t_rows); ++i)
            {
                writer.visitVectorElement(entry[i], param);
            }
            writer.endVector();
        }


        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            const EIGEN_DEFAULT_DENSE_INDEX_TYPE rows = entry.rows();
            const EIGEN_DEFAULT_DENSE_INDEX_TYPE cols = entry.cols();

            const bool dynamic = Eigen::Dynamic == t_rows or Eigen::Dynamic == t_cols;
            writer.startMatrix(dynamic, cols, rows, param);
            for (EIGEN_DEFAULT_DENSE_INDEX_TYPE i = 0; i < rows; ++i)
            {
                writer.startMatrixRow();
                for (EIGEN_DEFAULT_DENSE_INDEX_TYPE j = 0; j < cols; ++j)
                {
                    writer.visitMatrixElement(entry(i, j), param);
                }
                writer.endMatrixRow();
            }
            writer.endMatrix(dynamic);
        }


        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            apply_write(writer, entry.matrix(), param);
        }


        template <class t_Visitor, typename t_Scalar, int t_options, class t_Flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_write(
                t_Visitor &writer,
                const Eigen::Quaternion<t_Scalar, t_options> &entry,
                const t_Flags &param)
        {
            ARILES2_TRACE_FUNCTION;

            writer.startMap(param, 4);

            writer.visitMapEntry(entry.x(), "x", param);
            writer.visitMapEntry(entry.y(), "y", param);
            writer.visitMapEntry(entry.z(), "z", param);
            writer.visitMapEntry(entry.w(), "w", param);

            writer.endMap();
        }
    }  // namespace write
}  // namespace ariles2



namespace ariles2
{
    namespace compare
    {
        template <class t_Visitor, typename t_Scalar, int t_dim, int t_mode, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &left,
                const Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            visitor.equal_ &= (left.isApprox(right, param.template getTolerance<t_Scalar>()));
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const Eigen::Quaternion<t_Scalar, t_options> &left,
                const Eigen::Quaternion<t_Scalar, t_options> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            visitor.equal_ &= (left.isApprox(right, param.template getTolerance<t_Scalar>()));
        }


        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_compare(
                t_Visitor &visitor,
                const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &left,
                const Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            visitor.equal_ &= (left.isApprox(right, param.template getTolerance<t_Scalar>()));
        }
    }  // namespace compare
}  // namespace ariles2



namespace ariles2
{
    namespace defaults
    {
        template <class t_Visitor, typename t_Scalar, int t_rows, int t_cols, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                Eigen::Matrix<t_Scalar, t_rows, t_cols, t_flags> &entry,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
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
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                Eigen::Transform<t_Scalar, t_dim, t_mode, t_options> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            entry.setIdentity();
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_defaults(
                const t_Visitor & /*visitor*/,
                Eigen::Quaternion<t_Scalar, t_options> &entry,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            entry.setIdentity();
        }
    }  // namespace defaults
}  // namespace ariles2
