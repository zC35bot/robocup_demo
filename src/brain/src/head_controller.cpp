#include "head_controller.h"

#include <cmath>
#include <algorithm>

#include "brain.h"
#include "utils/math.h"

namespace {
constexpr double kUpdatePeriodSec = 0.02; // ~50Hz
}

void HeadController::update(Brain *brain) {
    double nowSec = brain->get_clock()->now().seconds();
    if (haveLastSend_ && (nowSec - lastSendSec_) < kUpdatePeriodSec) {
        return; // throttle to ~50Hz
    }

    Mode mode = selectMode(brain);

    // Track per-mode start times for phase-stable search patterns.
    if (mode != lastMode_) {
        if (mode == Mode::SEARCH_SWEEP) sweepStartSec_ = nowSec;
        if (mode == Mode::SEARCH_DIRECTED) directedStartSec_ = nowSec;
    }
    lastMode_ = mode;

    switch (mode) {
        case Mode::TRACK: {
            double pred100_robot[2];
            if (brain->data->using_field_frame) {
                Pose2D pf;
                pf.x = brain->data->pred100_field[0];
                pf.y = brain->data->pred100_field[1];
                pf.theta = 0;
                Pose2D pr = brain->data->field2robot(pf);
                pred100_robot[0] = pr.x;
                pred100_robot[1] = pr.y;
            } else {
                pred100_robot[0] = brain->data->pred100_robot[0];
                pred100_robot[1] = brain->data->pred100_robot[1];
            }
            trackBall(brain, pred100_robot);
            break;
        }
        case Mode::SEARCH_DIRECTED:
            searchDirected(brain, brain->data->ballVel);
            break;
        case Mode::SEARCH_SWEEP:
        default:
            searchSweep(brain);
            break;
    }
}

HeadController::Mode HeadController::selectMode(Brain *brain) const {
    if (brain->data->ball_prediction_valid) {
        return Mode::TRACK;
    }
    double sinceDetMs = brain->msecsSince(brain->data->ball.timePoint);
    double speed = norm(brain->data->ballVel[0], brain->data->ballVel[1]);
    if (sinceDetMs < 500.0 && speed > 0.1) {
        return Mode::SEARCH_DIRECTED;
    }
    return Mode::SEARCH_SWEEP;
}

void HeadController::trackBall(Brain *brain, const double pred100_robot[2]) {
    double x = pred100_robot[0];
    double y = pred100_robot[1];
    double range = norm(x, y);
    if (range < 1e-3) range = 1e-3;

    double yaw = atan2(y, x);
    double pitch = atan2(brain->config->get_robot_height(), range); // positive => look down

    sendHead(brain, pitch, yaw);
}

void HeadController::searchDirected(Brain *brain, const double vel_field[2]) {
    double nowSec = brain->get_clock()->now().seconds();
    double t = (directedStartSec_ >= 0.0) ? (nowSec - directedStartSec_) : 0.0;

    double dirField = atan2(vel_field[1], vel_field[0]);
    double centerYaw = toPInPI(dirField - brain->data->robotPoseToField.theta);

    const double amp = deg2rad(30.0);
    const double freq = 0.7; // Hz
    double yaw = centerYaw + amp * sin(2.0 * M_PI * freq * t);
    double pitch = 0.4;

    sendHead(brain, pitch, yaw);
}

void HeadController::searchSweep(Brain *brain) {
    double nowSec = brain->get_clock()->now().seconds();
    double t = (sweepStartSec_ >= 0.0) ? (nowSec - sweepStartSec_) : 0.0;

    const double leftYaw = brain->config->get_head_yaw_limit_left();
    const double rightYaw = brain->config->get_head_yaw_limit_right();
    const double lowPitch = 0.5;
    const double highPitch = 0.2;
    const double cycle = 2.0; // seconds for a full sweep (0.5Hz)

    double phase = fmod(t, cycle) / cycle; // [0,1)
    double tri = phase < 0.5 ? (phase * 2.0) : (2.0 - phase * 2.0); // 0->1->0
    double yaw = rightYaw + (leftYaw - rightYaw) * tri;
    double pitch = phase < 0.5 ? lowPitch : highPitch;

    sendHead(brain, pitch, yaw);
}

void HeadController::sendHead(Brain *brain, double pitch, double yaw) {
    const double leftYaw = brain->config->get_head_yaw_limit_left();
    const double rightYaw = brain->config->get_head_yaw_limit_right();
    yaw = cap(yaw, leftYaw, rightYaw);
    pitch = cap(pitch, 0.7, 0.0); // keep within a safe down/up band

    brain->client->moveHead(pitch, yaw);

    haveLastSend_ = true;
    lastSendSec_ = brain->get_clock()->now().seconds();
}
