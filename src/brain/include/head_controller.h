#pragma once

/**
 * @file head_controller.h
 * @brief Phase1 active head controller (spec §5).
 *
 * Drives the head at ~50Hz from Brain::tick(), reusing the existing robot_client
 * head interface (moveHead). It tracks the predicted ball (pred100) and performs
 * directed / sweeping search when the ball is not currently predictable.
 *
 * To avoid double-driving the head with the existing BT cam nodes
 * (CamTrackBall / CamScanField / CamFindBall), the controller only acts when
 * config head_controller.enable is true; in that mode the BT cam nodes are
 * short-circuited (see brain_tree.cpp). Default is disabled => behavior unchanged.
 */

class Brain; // forward declaration

class HeadController {
public:
    // Called every Brain::tick(); internally throttled to ~50Hz.
    void update(Brain *brain);

private:
    enum class Mode { TRACK, SEARCH_DIRECTED, SEARCH_SWEEP };

    Mode selectMode(Brain *brain) const;
    void trackBall(Brain *brain, const double pred100_robot[2]);
    void searchDirected(Brain *brain, const double vel_field[2]);
    void searchSweep(Brain *brain);

    void sendHead(Brain *brain, double pitch, double yaw);

    bool   haveLastSend_ = false;
    double lastSendSec_ = 0.0;
    double sweepStartSec_ = -1.0;
    double directedStartSec_ = -1.0;
    Mode   lastMode_ = Mode::SEARCH_SWEEP;
};
