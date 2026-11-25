#include "medhil/registration/registration.hpp"
#include <Eigen/SVD>
namespace medhil {
Result<Registration> register_points(const std::vector<Vec> &ct, const std::vector<Vec> &base,
                                     double max_fre_mm) {
    if (ct.size() < 3 || ct.size() != base.size() || !std::isfinite(max_fre_mm) || max_fre_mm < 0)
        return Error::InvalidInput;
    Vec a = Vec::Zero(), b = Vec::Zero();
    for (size_t i = 0; i < ct.size(); ++i) {
        if (!ct[i].allFinite() || !base[i].allFinite())
            return Error::InvalidInput;
        a += ct[i];
        b += base[i];
    }
    a /= ct.size();
    b /= ct.size();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < ct.size(); ++i)
        covariance += (ct[i] - a) * (base[i] - b).transpose();
    if (!covariance.allFinite())
        return Error::InvalidInput;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.rank() < 2)
        return Error::Degenerate;
    Eigen::Matrix3d correction = Eigen::Matrix3d::Identity();
    correction(2, 2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0 ? -1 : 1;
    Rigid t = Rigid::Identity();
    t.linear() = svd.matrixV() * correction * svd.matrixU().transpose();
    t.translation() = b - t.linear() * a;
    double sum = 0;
    for (size_t i = 0; i < ct.size(); ++i)
        sum += (t * ct[i] - base[i]).squaredNorm();
    double rms = 1000 * std::sqrt(sum / ct.size());
    if (!std::isfinite(rms) || rms > max_fre_mm)
        return Error::PoorFit;
    return Registration{pose(t, Frame::PatientCT, Frame::RobotBase), rms};
}
} // namespace medhil
