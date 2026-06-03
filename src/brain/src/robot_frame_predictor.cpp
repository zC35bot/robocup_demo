#include "robot_frame_predictor.h"

#include <cmath>
#include <algorithm>

namespace {
constexpr double kEps = 1e-9;
}

RobotFramePredictor::RobotFramePredictor() {
    reset();
}

void RobotFramePredictor::reset() {
    x_.setZero();
    P_ = Eigen::Matrix4d::Identity();
    mu_ = std::max(0.0, params_.friction_decay_hz);
    initialized_ = false;
    frames_since_detection_ = 0;
    confidence_ = 0.0;
    occ_q_scale_ = 1.0;
}

Eigen::Matrix2d RobotFramePredictor::measNoise(double detDist) {
    double sigma = 0.124 * detDist + 0.149;
    if (sigma < 1e-3) sigma = 1e-3;
    return (sigma * sigma) * Eigen::Matrix2d::Identity();
}

void RobotFramePredictor::propagate(double dt) {
    if (!initialized_ || dt <= 0.0) return;
    mu_ = std::max(0.0, params_.friction_decay_hz);

    double decay = 1.0 - mu_ * dt;
    if (decay < 0.0) decay = 0.0;

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = dt;
    F(1, 3) = dt;
    F(2, 2) = decay;
    F(3, 3) = decay;
    x_ = F * x_;

    const double s2 = sigmaA_ * sigmaA_ * occ_q_scale_;
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
    P_ = F * P_ * F.transpose() + Q;
}

void RobotFramePredictor::add(const Eigen::Vector2d &z, double detDist) {
    Eigen::Matrix2d R = measNoise(detDist);
    if (!initialized_) {
        x_ << z(0), z(1), 0.0, 0.0;
        P_ = Eigen::Matrix4d::Identity();
        P_(0, 0) = R(0, 0);
        P_(1, 1) = R(1, 1);
        P_(2, 2) = 4.0;
        P_(3, 3) = 4.0;
        initialized_ = true;
        frames_since_detection_ = 0;
        confidence_ = 1.0;
        occ_q_scale_ = 1.0;
        return;
    }

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    Eigen::Vector2d y = z - H * x_;
    Eigen::Matrix2d S = H * P_ * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = P_ * H.transpose() * S.inverse();
    x_ = x_ + K * y;
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P_ = (I - K * H) * P_;

    frames_since_detection_ = 0;
    confidence_ = 1.0;
    occ_q_scale_ = 1.0;
}

void RobotFramePredictor::handleOccluded() {
    if (!initialized_) return;
    frames_since_detection_++;
    occ_q_scale_ *= std::max(1.0, params_.occlusion_noise_growth);
    confidence_ *= params_.confidence_decay_rate;
    if (frames_since_detection_ > params_.max_occluded_frames) {
        confidence_ = 0.0;
    }
}

BallPrediction RobotFramePredictor::getPrediction() const {
    BallPrediction out{};
    double px = x_(0), py = x_(1), vx = x_(2), vy = x_(3);
    out.pos[0] = static_cast<float>(px);
    out.pos[1] = static_cast<float>(py);
    out.vel[0] = static_cast<float>(vx);
    out.vel[1] = static_cast<float>(vy);
    out.mode_prob[0] = 0.0f;
    out.mode_prob[1] = 1.0f;
    out.confidence = static_cast<float>(std::max(0.0, std::min(1.0, confidence_)));

    bool valid = initialized_ && (frames_since_detection_ <= params_.max_occluded_frames) && (confidence_ > 1e-3);
    out.valid = valid;
    out.pred300_valid = false; // robot-frame fallback: no pred300

    double mu = std::max(0.0, params_.friction_decay_hz);
    auto extrapolate = [&](double t, float o[2]) {
        double fx = vx * t - 0.5 * mu * vx * t * t;
        double fy = vy * t - 0.5 * mu * vy * t * t;
        if (fx * vx < 0.0) fx = 0.0;
        if (fy * vy < 0.0) fy = 0.0;
        o[0] = static_cast<float>(px + fx);
        o[1] = static_cast<float>(py + fy);
    };
    extrapolate(0.1, out.pred100);
    extrapolate(0.3, out.pred300); // computed but pred300_valid=false; not consumed downstream

    return out;
}
