#pragma once

/**
 * @file pos_predictor.h
 * @brief Phase1 ball state estimation: a field-frame IMM-EKF (BallImmPredictor).
 *
 * Two dynamic models are mixed by an Interacting-Multiple-Model (IMM) filter:
 *   - stationary : ball at rest / near-rest (large friction, small process noise)
 *   - rolling    : uniformly decelerating roll (small friction, medium process noise)
 *
 * State x = [px, py, vx, vy]^T in the field frame.
 * Measurement z = ball center position in field frame, H selects position only.
 * Measurement noise R(d) = (0.124*d + 0.149)^2 * I2, d = detection distance.
 *
 * The predictor exposes pred100 (100ms ahead, Chase main path) and pred300
 * (300ms ahead, Adjust/Kick) plus an occlusion-decayed confidence and the
 * Mahalanobis-distance helper used by detectProcessBalls gating.
 *
 * NOTE: dynamics and the position-only measurement are linear, so the EKF
 * reduces to a linear KF per model; we keep the EKF naming per the spec.
 */

#include <Eigen/Dense>
#include <array>

// Output of the predictor, consumed by brain/BT (see spec §3.4).
struct BallPrediction {
    float pos[2];        // field frame current filtered position
    float vel[2];        // current filtered velocity
    float pred100[2];    // 100ms prediction (Chase main path)
    float pred300[2];    // 300ms prediction (Kick/Adjust)
    float confidence;    // [0,1], decays during occlusion
    float mode_prob[2];  // [stationary, rolling]
    bool  valid;
    bool  pred300_valid; // only true on the field main path
};

// Tunables (loaded from config ball_predictor.* in brain).
struct BallPredictorParams {
    double friction_decay_hz = 0.30;     // rolling friction decay (s^-1)
    double occlusion_noise_growth = 1.05; // per-frame Q growth while occluded
    int    max_occluded_frames = 50;     // invalidate prediction after this many occluded frames
    double confidence_decay_rate = 0.92; // per-frame confidence decay while occluded
    double gate_chi2_threshold = 5.99;   // chi2_0.95(2)
    double jump_dist_m_per_frame = 0.40; // jump-reject distance threshold (used by brain)
};

class BallImmPredictor {
public:
    BallImmPredictor();

    void configure(const BallPredictorParams &p) { params_ = p; }
    const BallPredictorParams &params() const { return params_; }

    void reset();

    // Time update: propagate all models by dt seconds (IMM mixing + per-model predict).
    void propagate(double dt);

    // Measurement update with a field-frame position; detDist sets R(d).
    void add(const Eigen::Vector2d &z, double detDist);

    // Occlusion: inflate process noise, decay confidence, count frames, invalidate when stale.
    void handleOccluded();

    // Current fused prediction.
    BallPrediction getPrediction() const;

    bool initialized() const { return initialized_; }
    int  framesSinceDetection() const { return frames_since_detection_; }
    double confidence() const { return confidence_; }

    // Mahalanobis distance of measurement z given the fused state and R(detDist).
    // Returns a large value when not yet initialized (so first measurement always passes).
    double mahalanobis(const Eigen::Vector2d &z, double detDist) const;

private:
    struct Model {
        Eigen::Vector4d x = Eigen::Vector4d::Zero();
        Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
        double mu = 0.0;     // friction decay (s^-1)
        double sigmaA = 0.0; // process accel std-dev
        double prob = 0.5;   // model probability

        void predict(double dt, double qScale);
        // returns measurement likelihood N(innovation; 0, S)
        double update(const Eigen::Vector2d &z, const Eigen::Matrix2d &R);
    };

    void fuse(Eigen::Vector4d &x, Eigen::Matrix4d &P) const;
    static Eigen::Matrix2d measNoise(double detDist);

    static constexpr int NMODELS = 2; // 0 = stationary, 1 = rolling
    std::array<Model, NMODELS> models_;
    Eigen::Matrix<double, NMODELS, NMODELS> transMat_; // Markov transition probabilities
    Eigen::Matrix<double, NMODELS, 1> cbar_;           // predicted (mixed) mode probs

    BallPredictorParams params_;
    bool initialized_ = false;
    int frames_since_detection_ = 0;
    double confidence_ = 0.0;
    double occ_q_scale_ = 1.0; // accumulated occlusion noise growth
};
