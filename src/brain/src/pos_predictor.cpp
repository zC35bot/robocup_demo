#include "pos_predictor.h"

#include <cmath>
#include <algorithm>

namespace {
constexpr double kEps = 1e-9;
}

// ----------------------------- Model -----------------------------------------

void BallImmPredictor::Model::predict(double dt, double qScale) {
    if (dt <= 0.0) return;

    // State transition with friction on velocity (clamped to keep it stable / non-reversing).
    double decay = 1.0 - mu * dt;
    if (decay < 0.0) decay = 0.0;

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    F(2, 2) = decay;
    F(3, 3) = decay;

    x = F * x;

    // White-noise acceleration process noise.
    const double s2 = sigmaA * sigmaA * qScale;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = dt4 / 4.0 * s2;
    Q(1, 1) = dt4 / 4.0 * s2;
    Q(0, 2) = dt3 / 2.0 * s2;
    Q(2, 0) = dt3 / 2.0 * s2;
    Q(1, 3) = dt3 / 2.0 * s2;
    Q(3, 1) = dt3 / 2.0 * s2;
    Q(2, 2) = dt2 * s2;
    Q(3, 3) = dt2 * s2;

    P = F * P * F.transpose() + Q;
}

double BallImmPredictor::Model::update(const Eigen::Vector2d &z, const Eigen::Matrix2d &R) {
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;

    Eigen::Vector2d y = z - H * x;          // innovation
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix2d Sinv = S.inverse();
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * Sinv;

    x = x + K * y;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P = (I - K * H) * P;

    // Gaussian likelihood of the innovation.
    double det = S.determinant();
    if (det < kEps) det = kEps;
    double md2 = (y.transpose() * Sinv * y)(0, 0);
    double like = std::exp(-0.5 * md2) / std::sqrt(4.0 * M_PI * M_PI * det);
    if (!std::isfinite(like) || like < kEps) like = kEps;
    return like;
}

// ----------------------------- BallImmPredictor ------------------------------

BallImmPredictor::BallImmPredictor() {
    transMat_ << 0.95, 0.05,
                 0.05, 0.95;
    cbar_.setConstant(1.0 / NMODELS);
    reset();
}

void BallImmPredictor::reset() {
    // model 0: stationary (strong friction, tiny accel noise)
    // model 1: rolling   (light friction, medium accel noise)
    models_[0].mu = 6.0;
    models_[0].sigmaA = 0.05;
    models_[0].prob = 0.5;
    models_[0].x.setZero();
    models_[0].P = Eigen::Matrix4d::Identity();

    models_[1].mu = std::max(0.0, params_.friction_decay_hz);
    models_[1].sigmaA = 0.6;
    models_[1].prob = 0.5;
    models_[1].x.setZero();
    models_[1].P = Eigen::Matrix4d::Identity();

    cbar_.setConstant(1.0 / NMODELS);
    initialized_ = false;
    frames_since_detection_ = 0;
    confidence_ = 0.0;
    occ_q_scale_ = 1.0;
}

Eigen::Matrix2d BallImmPredictor::measNoise(double detDist) {
    double sigma = 0.124 * detDist + 0.149;
    if (sigma < 1e-3) sigma = 1e-3;
    return (sigma * sigma) * Eigen::Matrix2d::Identity();
}

void BallImmPredictor::propagate(double dt) {
    if (!initialized_ || dt <= 0.0) return;
    // keep rolling model friction in sync with live config
    models_[1].mu = std::max(0.0, params_.friction_decay_hz);

    // ---- IMM mixing ----
    Eigen::Matrix<double, NMODELS, 1> mu;
    for (int i = 0; i < NMODELS; ++i) mu(i) = models_[i].prob;

    // predicted mode probabilities c_j = sum_i pi_ij * mu_i
    for (int j = 0; j < NMODELS; ++j) {
        double c = 0.0;
        for (int i = 0; i < NMODELS; ++i) c += transMat_(i, j) * mu(i);
        cbar_(j) = std::max(c, kEps);
    }

    // mixing weights w_ij = pi_ij * mu_i / c_j
    std::array<Eigen::Vector4d, NMODELS> xMixed;
    std::array<Eigen::Matrix4d, NMODELS> PMixed;
    for (int j = 0; j < NMODELS; ++j) {
        Eigen::Vector4d xm = Eigen::Vector4d::Zero();
        for (int i = 0; i < NMODELS; ++i) {
            double w = transMat_(i, j) * mu(i) / cbar_(j);
            xm += w * models_[i].x;
        }
        Eigen::Matrix4d Pm = Eigen::Matrix4d::Zero();
        for (int i = 0; i < NMODELS; ++i) {
            double w = transMat_(i, j) * mu(i) / cbar_(j);
            Eigen::Vector4d d = models_[i].x - xm;
            Pm += w * (models_[i].P + d * d.transpose());
        }
        xMixed[j] = xm;
        PMixed[j] = Pm;
    }

    for (int j = 0; j < NMODELS; ++j) {
        models_[j].x = xMixed[j];
        models_[j].P = PMixed[j];
    }

    // ---- per-model prediction ----
    for (int j = 0; j < NMODELS; ++j) {
        models_[j].predict(dt, occ_q_scale_);
    }
}

void BallImmPredictor::add(const Eigen::Vector2d &z, double detDist) {
    Eigen::Matrix2d R = measNoise(detDist);

    if (!initialized_) {
        for (int j = 0; j < NMODELS; ++j) {
            models_[j].x << z(0), z(1), 0.0, 0.0;
            models_[j].P = Eigen::Matrix4d::Identity();
            models_[j].P(0, 0) = R(0, 0);
            models_[j].P(1, 1) = R(1, 1);
            models_[j].P(2, 2) = 4.0; // unknown initial velocity
            models_[j].P(3, 3) = 4.0;
            models_[j].prob = 1.0 / NMODELS;
        }
        cbar_.setConstant(1.0 / NMODELS);
        initialized_ = true;
        frames_since_detection_ = 0;
        confidence_ = 1.0;
        occ_q_scale_ = 1.0;
        return;
    }

    // ---- per-model measurement update + likelihood ----
    Eigen::Matrix<double, NMODELS, 1> likelihood;
    for (int j = 0; j < NMODELS; ++j) {
        likelihood(j) = models_[j].update(z, R);
    }

    // ---- mode probability update: mu_j = L_j * cbar_j / sum ----
    double norm = 0.0;
    for (int j = 0; j < NMODELS; ++j) norm += likelihood(j) * cbar_(j);
    if (norm < kEps) norm = kEps;
    for (int j = 0; j < NMODELS; ++j) {
        models_[j].prob = likelihood(j) * cbar_(j) / norm;
    }

    frames_since_detection_ = 0;
    confidence_ = 1.0;
    occ_q_scale_ = 1.0;
}

void BallImmPredictor::handleOccluded() {
    if (!initialized_) return;
    frames_since_detection_++;
    occ_q_scale_ *= std::max(1.0, params_.occlusion_noise_growth);
    confidence_ *= params_.confidence_decay_rate;
    if (frames_since_detection_ > params_.max_occluded_frames) {
        confidence_ = 0.0;
    }
}

void BallImmPredictor::fuse(Eigen::Vector4d &x, Eigen::Matrix4d &P) const {
    x.setZero();
    for (int j = 0; j < NMODELS; ++j) x += models_[j].prob * models_[j].x;
    P.setZero();
    for (int j = 0; j < NMODELS; ++j) {
        Eigen::Vector4d d = models_[j].x - x;
        P += models_[j].prob * (models_[j].P + d * d.transpose());
    }
}

double BallImmPredictor::mahalanobis(const Eigen::Vector2d &z, double detDist) const {
    if (!initialized_) return 0.0; // first measurement always passes
    Eigen::Vector4d x;
    Eigen::Matrix4d P;
    fuse(x, P);
    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    Eigen::Vector2d y = z - H * x;
    Eigen::Matrix2d S = H * P * H.transpose() + measNoise(detDist);
    double md2 = (y.transpose() * S.inverse() * y)(0, 0);
    if (md2 < 0.0) md2 = 0.0;
    return std::sqrt(md2);
}

BallPrediction BallImmPredictor::getPrediction() const {
    BallPrediction out{};
    Eigen::Vector4d x;
    Eigen::Matrix4d P;
    if (initialized_) {
        fuse(x, P);
    } else {
        x.setZero();
        P.setIdentity();
    }

    double px = x(0), py = x(1), vx = x(2), vy = x(3);
    double pStat = models_[0].prob;
    double pRoll = models_[1].prob;

    out.pos[0] = static_cast<float>(px);
    out.pos[1] = static_cast<float>(py);
    out.vel[0] = static_cast<float>(vx);
    out.vel[1] = static_cast<float>(vy);
    out.mode_prob[0] = static_cast<float>(pStat);
    out.mode_prob[1] = static_cast<float>(pRoll);
    out.confidence = static_cast<float>(std::max(0.0, std::min(1.0, confidence_)));

    bool valid = initialized_ && (frames_since_detection_ <= params_.max_occluded_frames) && (confidence_ > 1e-3);
    out.valid = valid;
    out.pred300_valid = valid; // field main path provides pred300

    // Horizon extrapolation. When rolling dominates use the friction model,
    // otherwise (stationary) return the current position.
    auto extrapolate = [&](double t, float out2[2]) {
        if (pRoll >= pStat) {
            double mu = std::max(0.0, params_.friction_decay_hz);
            // p(t) = p + v*t - 0.5*mu*v*t^2 (displacement shrinks with friction, never reverses)
            double fx = vx * t - 0.5 * mu * vx * t * t;
            double fy = vy * t - 0.5 * mu * vy * t * t;
            // clamp so the friction term cannot flip the sign of the displacement
            if (fx * vx < 0.0) fx = 0.0;
            if (fy * vy < 0.0) fy = 0.0;
            out2[0] = static_cast<float>(px + fx);
            out2[1] = static_cast<float>(py + fy);
        } else {
            out2[0] = static_cast<float>(px);
            out2[1] = static_cast<float>(py);
        }
    };
    extrapolate(0.1, out.pred100);
    extrapolate(0.3, out.pred300);

    return out;
}
