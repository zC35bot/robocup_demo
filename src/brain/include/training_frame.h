#pragma once

/**
 * @file training_frame.h
 * @brief Phase1 per-frame training sample (spec §9), aligned with real robocup_demo quantities.
 */

#include <cstdint>

struct TrainingFrame {
    uint64_t timestamp_us = 0;

    float raw_ball_robot[2] = {0, 0};      // data->ball.posToRobot
    float raw_ball_conf = 0;               // data->ball.confidence (0..100 scale)
    float bbox_xywh[4] = {0, 0, 0, 0};     // data->ball.boundingBox (x,y,w,h)

    float filtered_ball_field[2] = {0, 0}; // imm filtered position
    float pred100_field[2] = {0, 0};
    float pred300_field[2] = {0, 0};
    uint8_t pred300_valid = 0;
    uint8_t using_field_frame = 0;

    float pred100_robot[2] = {0, 0};
    float mode_prob[2] = {0, 0};
    float ball_confidence = 0;             // predictor confidence [0,1]

    float robot_pose[3] = {0, 0, 0};       // data->robotPoseToField (x,y,theta)
    float robot_vel[3] = {0, 0, 0};        // odometry derived (vx,vy,vtheta)
    float head_pose[2] = {0, 0};           // (yaw, pitch)
    float imu_acc[3] = {0, 0, 0};          // lowState acc

    uint8_t player_role = 0;               // 1 striker, 2 goal_keeper, 3 unknown
    uint8_t decision = 0;                  // encoded decision id
    uint8_t is_lead = 0;
    float cost = 0;                        // tmMyCost

    int8_t kick_result = -1;               // post-labeled
    uint8_t abort_reason = 0;              // post-labeled

    float tm_age_ms[4] = {0, 0, 0, 0};

    uint8_t fall_state = 0;                // recoveryState
    uint8_t game_state = 0;                // encoded gc_game_state
};
