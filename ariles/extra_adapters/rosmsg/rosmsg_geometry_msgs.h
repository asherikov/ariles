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

namespace ariles2
{
    namespace copyfrom
    {
        template <class t_Visitor, typename t_Scalar, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                t_Visitor & /*visitor*/,
                Eigen::Matrix<t_Scalar, 3, 1, t_flags> &left,
                const geometry_msgs::Vector3 &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            left.x() = right.x;
            left.y() = right.y;
            left.z() = right.z;
        }

        template <class t_Visitor, typename t_Scalar, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                t_Visitor & /*visitor*/,
                Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1, t_flags> &left,
                const geometry_msgs::Vector3 &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            left.resize(3);
            left(0) = right.x;
            left(1) = right.y;
            left(2) = right.z;
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                const t_Visitor & /*visitor*/,
                Eigen::Quaternion<t_Scalar, t_options> &left,
                const geometry_msgs::Quaternion &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            left.x() = right.x;
            left.y() = right.y;
            left.z() = right.z;
            left.w() = right.w;
        }


        template <class t_Visitor, typename t_Scalar, int t_mode, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyfrom(
                const t_Visitor &visitor,
                Eigen::Transform<t_Scalar, 3, t_mode, t_options> &left,
                const geometry_msgs::Transform &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;

            Eigen::Quaternion<t_Scalar> quaternion;
            Eigen::Matrix<t_Scalar, 3, 1> translation;

            apply_copyfrom(visitor, quaternion, right.rotation, param);
            apply_copyfrom(visitor, translation, right.translation, param);

            left(3, 3) = 1.0;
            left.linear() = quaternion.toRotationMatrix();
            left.translation() = translation;
        }
    }  // namespace copyfrom


    namespace copyto
    {
        template <class t_Visitor, typename t_Scalar, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor & /*visitor*/,
                const Eigen::Matrix<t_Scalar, 3, 1, t_flags> &left,
                geometry_msgs::Vector3 &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            right.x = left.x();
            right.y = left.y();
            right.z = left.z();
        }

        template <class t_Visitor, typename t_XprType, bool t_InnerPanel>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor & /*visitor*/,
                const Eigen::Block<t_XprType, 3, 1, t_InnerPanel> &left,
                geometry_msgs::Vector3 &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            right.x = left(0);
            right.y = left(1);
            right.z = left(2);
        }

        template <class t_Visitor, typename t_Scalar, int t_flags>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                t_Visitor & /*visitor*/,
                const Eigen::Matrix<t_Scalar, Eigen::Dynamic, 1, t_flags> &left,
                geometry_msgs::Vector3 &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            ARILES2_ASSERT(3 == left.size(), "Wrong entry size.");
            right.x = left(0);
            right.y = left(1);
            right.z = left(2);
        }


        template <class t_Visitor, typename t_Scalar, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                const t_Visitor & /*visitor*/,
                const Eigen::Quaternion<t_Scalar, t_options> &left,
                geometry_msgs::Quaternion &right,
                const typename t_Visitor::Parameters & /*param*/)
        {
            ARILES2_TRACE_FUNCTION;
            right.x = left.x();
            right.y = left.y();
            right.z = left.z();
            right.w = left.w();
        }


        template <class t_Visitor, typename t_Scalar, int t_mode, int t_options>
        void ARILES2_VISIBILITY_ATTRIBUTE apply_copyto(
                const t_Visitor &visitor,
                const Eigen::Transform<t_Scalar, 3, t_mode, t_options> &left,
                geometry_msgs::Transform &right,
                const typename t_Visitor::Parameters &param)
        {
            ARILES2_TRACE_FUNCTION;
            apply_copyto(visitor, Eigen::Quaternion<t_Scalar>(left.linear()), right.rotation, param);
            apply_copyto(visitor, left.translation(), right.translation, param);
        }
    }  // namespace copyto
}  // namespace ariles2
