#pragma once

/**
 * @file robot_frame_predictor.h
 * @brief Phase1 short-range ball predictor in the robot frame (localization-free fallback).
 *
 * A single constant-velocity-with-friction KF over x = [px, py, vx, vy] in the
 * robot frame. It does not depend on field localization, so it stays usable when
 * localization is not trusted. It only provides pred100 (Chase fallback) and a
 * validity flag (no pred300, see spec §3.4 / §3.2).
 */

#include <Eigen/Dense>
#include "pos_predictor.h" // for BallPrediction / BallPredictorParams reuse

class RobotFramePredictor {
public:
    RobotFramePredictor();

    void configure(const BallPredictorParams &p) { params_ = p; }
    void reset();

    void propagate(double dt);
    void add(const Eigen::Vector2d &z, double detDist); // z in robot frame
    void handleOccluded();

    BallPrediction getPrediction() const;

    bool initialized() const { return initialized_; }
    int  framesSinceDetection() const { return frames_since_detection_; }

private:
    static Eigen::Matrix2d measNoise(double detDist);

    Eigen::Vector4d x_ = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P_ = Eigen::Matrix4d::Identity();
    double mu_ = 0.30;     // friction decay (s^-1)
    double sigmaA_ = 0.8;  // process accel std-dev (robot-frame is noisier: robot moves)

    BallPredictorParams params_;
    bool initialized_ = false;
    int frames_since_detection_ = 0;
    double confidence_ = 0.0;
    double occ_q_scale_ = 1.0;
};
