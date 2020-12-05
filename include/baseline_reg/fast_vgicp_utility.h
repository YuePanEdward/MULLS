//Copyright (c) 2019, k.koide

#ifndef FAST_VGICP_UTILITY_H
#define FAST_VGICP_UTILITY_H

#include <Eigen/Core>

namespace koide_reg
{

    enum RegularizationMethod
    {
        NONE,
        MIN_EIG,
        NORMALIZED_MIN_EIG,
        PLANE,
        FROBENIUS
    };

    enum VoxelAccumulationMode
    {
        ADDITIVE,
        ADDITIVE_WEIGHTED,
        MULTIPLICATIVE
    };

    inline Eigen::Matrix3f skew(const Eigen::Vector3f &x)
    {
        Eigen::Matrix3f skew = Eigen::Matrix3f::Zero();
        skew(0, 1) = -x[2];
        skew(0, 2) = x[1];
        skew(1, 0) = x[2];
        skew(1, 2) = -x[0];
        skew(2, 0) = -x[1];
        skew(2, 1) = x[0];

        return skew;
    }

    template <typename Scalar, int N>
    class GaussNewton
    {
    public:
        GaussNewton() {}
        virtual ~GaussNewton() {}

        virtual Eigen::Matrix<Scalar, N, 1> delta(const Eigen::Matrix<Scalar, -1, 1> &e, const Eigen::Matrix<Scalar, -1, -1> &J) const
        {
            Eigen::Matrix<Scalar, N, N> JJ = J.transpose() * J;
            Eigen::Matrix<Scalar, N, 1> delta = JJ.llt().solve(J.transpose() * e);

            if (!delta.array().isFinite().all())
            {
                // std::cerr << "!!!! delta corrupted !!!!" << std::endl;
                return Eigen::Matrix<Scalar, N, 1>::Random() * 1e-2;
            }

            return delta;
        }
    };

    // WIP
    template <typename Scalar, int N>
    class LevenbergMarquardt : public GaussNewton<Scalar, N>
    {
    public:
        LevenbergMarquardt(double init_lambda = 10.0)
            : lambda(init_lambda)
        {
        }
        virtual ~LevenbergMarquardt() override {}

        virtual Eigen::Matrix<Scalar, N, 1> delta(const Eigen::Matrix<Scalar, -1, 1> &e, const Eigen::Matrix<Scalar, -1, -1> &J) const override
        {
            Eigen::Matrix<Scalar, N, N> JJ = J.transpose() * J;

            for (int i = 0; i < N; i++)
            {
                JJ(i, i) = JJ(i, i) + lambda * J(i, i);
            }

            Eigen::Matrix<Scalar, N, 1> delta = JJ.llt().solve(10.0 * J.transpose() * e);
            return delta;
        }

    private:
        double lambda;
    };

} // namespace koide_reg

#endif