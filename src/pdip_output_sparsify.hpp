#ifndef REGOT_PDIP_OUTPUT_SPARSIFY_H
#define REGOT_PDIP_OUTPUT_SPARSIFY_H

#include "pdip_result.h"
#include "pdip_transport_ops.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <Eigen/Core>
#include <vector>

namespace PDIP {
namespace output_sparsify {

static const double kSmall = 1e-50;
static const double kPrimaryXScale = 1e-3;
static const double kFallbackXScale = 1e-5;
static const double kSxRatioMin = 1000.0;

struct PdipGaps {
    double obj = 0, primal_gap = 0, dual_gap = 0, mu = 0;
};

inline PdipGaps compute_pdip_gaps(
    int n, int m, double barrier, double reg_val,
    const double* x, const double* s, const double* lambda_val,
    const double* cost_vec, const double* eq_vec
) {
    using Vector = Eigen::VectorXd;
    const int n_vars = n * m;
    const int n_constraints = m + n - 1;
    Vector at_lambda(n_vars), r_dual(n_vars), r_pri(n_constraints);
    Eigen::Map<const Vector> xv(x, n_vars);
    Eigen::Map<const Vector> sv(s, n_vars);
    Eigen::Map<const Vector> cost(cost_vec, n_vars);
    Eigen::Map<const Vector> eq(eq_vec, n_constraints);
    transport::AT_matvec(n, m, lambda_val, at_lambda.data());
    r_dual = barrier * xv + cost + at_lambda - sv;
    transport::A_matvec_from_x(n, m, x, nullptr, r_pri.data());
    r_pri.head(m) -= eq.head(m);
    r_pri.tail(n - 1) -= eq.tail(n - 1);
    PdipGaps g;
    g.primal_gap = r_pri.norm() / (1.0 + eq.norm());
    g.dual_gap = r_dual.norm() / (1.0 + cost.norm() + at_lambda.norm());
    g.mu = xv.dot(sv) / static_cast<double>(n_vars);
    g.obj = cost.dot(xv) + (reg_val / 2.0) * xv.squaredNorm();
    return g;
}

// Zero x[i] when s/x > kSxRatioMin and x < x_scale/n (n = source dimension).
inline int sparsify_x(double* x, const double* s, int n_vars, int n, double x_scale) {
    const double x_thresh = x_scale / static_cast<double>((std::max)(n, 1));
    int n_zeroed = 0;
    for (int i = 0; i < n_vars; ++i) {
        if (x[i] > 0 && x[i] < x_thresh && s[i] / (x[i] + kSmall) > kSxRatioMin) {
            x[i] = 0.0;
            ++n_zeroed;
        }
    }
    return n_zeroed;
}

inline bool sparsify_quality_poor(const PdipGaps& before, const PdipGaps& after, double tol) {
    const double pg_lim = (std::max)(tol, 100.0 * before.primal_gap);
    if (after.primal_gap > pg_lim) return true;
    const double rel_obj = std::abs(after.obj - before.obj) / (std::abs(before.obj) + 1e-20);
    if (rel_obj > 1e-2) return true;
    return false;
}

inline void apply_output_sparsify(
    PDIPResult& result, int n, int m, double barrier, double reg_val, double tol,
    double* x, const double* s, const double* lambda_val,
    const double* cost_vec, const double* eq_vec
) {
    const int n_vars = n * m;
    const PdipGaps before = compute_pdip_gaps(
        n, m, barrier, reg_val, x, s, lambda_val, cost_vec, eq_vec);
    std::vector<double> x_backup(static_cast<size_t>(n_vars));
    std::memcpy(x_backup.data(), x, static_cast<size_t>(n_vars) * sizeof(double));

    result.sparsify_applied = true;
    result.pre_sparsify_obj = before.obj;
    result.pre_sparsify_primal_gap = before.primal_gap;
    result.pre_sparsify_dual_gap = before.dual_gap;
    result.pre_sparsify_mu = before.mu;

    double x_scale_used = kPrimaryXScale;
    std::memcpy(x, x_backup.data(), static_cast<size_t>(n_vars) * sizeof(double));
    int n_zeroed = sparsify_x(x, s, n_vars, n, kPrimaryXScale);
    PdipGaps after = compute_pdip_gaps(
        n, m, barrier, reg_val, x, s, lambda_val, cost_vec, eq_vec);

    result.sparsify_rollback = false;
    if (sparsify_quality_poor(before, after, tol)) {
        std::memcpy(x, x_backup.data(), static_cast<size_t>(n_vars) * sizeof(double));
        n_zeroed = sparsify_x(x, s, n_vars, n, kFallbackXScale);
        after = compute_pdip_gaps(
            n, m, barrier, reg_val, x, s, lambda_val, cost_vec, eq_vec);
        x_scale_used = kFallbackXScale;
        result.sparsify_rollback = true;
    }

    result.sparsify_x_scale = x_scale_used;
    result.sparsify_n_zeroed = n_zeroed;
    result.post_sparsify_obj = after.obj;
    result.post_sparsify_primal_gap = after.primal_gap;
    result.post_sparsify_dual_gap = after.dual_gap;
    result.post_sparsify_mu = after.mu;
}

}  // namespace output_sparsify
}  // namespace PDIP

#endif  // REGOT_PDIP_OUTPUT_SPARSIFY_H
