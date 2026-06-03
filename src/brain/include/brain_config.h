#pragma once

#include <string>
#include <ostream>
#include <rclcpp/rclcpp.hpp>

#include "types.h"
#include "utils/math.h"
#include "RoboCupGameControlData.h"

const std::string GAME_AGENT_MODE_PARAM = "game.agent_mode";

class Brain; // forward declaration

using namespace std;

/* ---------------- Phase1 parameter blocks (see spec §10) ---------------- */

struct Phase1BallPredictorConfig {
    double friction_decay_hz = 0.30;
    double occlusion_noise_growth = 1.05;
    int    max_occluded_frames = 50;
    double ball_confidence_decay_rate = 0.92;
    double gate_chi2_threshold = 5.99;
    double jump_dist_m_per_frame = 0.40;
    double localization_trust_msec = 1500.0;
    double localization_trust_cov_max = 0.25;
};

struct Phase1ChaseConfig {
    double predict_msec = 100.0;
};

struct Phase1KickConfig {
    double abort_ball_move_dist_m = 0.15;
    double abort_confidence = 0.35;
    double abort_yaw_deg = 20.0;
};

struct Phase1KickSelectorConfig {
    double abort_confidence = 0.25;
    double adjust_yaw_deg = 15.0;
    double rlvision_range_max_m = 0.40;
    double rlvision_yaw_deg = 8.0;
    double rlvision_conf_min = 0.50;
};

struct Phase1PowerShootConfig {
    bool   enable = true;
    double min_dist_to_goal_m = 1.5;
    double max_dist_to_goal_m = 4.0;
    double min_goal_angle_deg = 15.0;
    double robot_stability_min = 0.8;
};

struct Phase1CooperationConfig {
    double handover_margin_m = 0.30;
    double lead_min_msec = 800.0;
    double stale_threshold_ms = 500.0;
};

struct Phase1HeadControllerConfig {
    bool enable = false;
};

struct Phase1TrainingLoggerConfig {
    bool enable = false;
};

/**
 * Stores configuration values required by `Brain`. These values should be
 * determined at initialization and treated as read-only during decision-making.
 * Values that change during runtime should be placed in `BrainData`.
 * Notes:
 * 1) Configuration is read from `config/config.yaml`.
 * 2) If `config/config_local.yaml` exists it overrides `config/config.yaml`.
 */
class BrainConfig
{
public:
    // Pointer to ROS2 node used to access parameters
    BrainConfig(Brain *argBrain);

    int get_player_id();
    int get_team_id();
    string get_player_role();
    string get_field_type();   
    int get_num_of_players();
    bool get_treat_person_as_robot();
    string get_game_control_ip();

    string get_robot_name();
    double get_robot_height();        
    double get_robot_odom_factor();
    double get_vx_factor();
    double get_yaw_offset();
    double get_vx_limit();
    double get_vy_limit();
    double get_vtheta_limit();
    double get_head_yaw_limit_left();
    double get_head_yaw_limit_right();
    double get_head_pitch_limit_up();
    double get_min_vx();
    double get_min_vy();
    double get_min_vtheta();

    double get_ball_confidence_threshold();
    double get_ball_memory_timeout();
    double get_tm_ball_dist_threshold();
    bool get_limit_near_ball_speed();
    double get_near_ball_speed_limit();
    double get_near_ball_range();
    double get_ball_out_threshold();
    bool get_abort_kick_when_ball_moved();
    bool get_enable_role_switch();
    double get_ball_control_cost_threshold();

    bool get_enable_auto_visual_kick();
    double get_auto_visual_kick_enable_dist_min();
    double get_auto_visual_kick_enable_dist_max();
    double get_auto_visual_kick_enable_angle();

    int get_min_marker_count();
    double get_max_residual();


    bool get_enable_com();

    string get_tree_file_path();

    int get_depth_sample_step();
    double get_obstacle_min_height();
    double get_grid_size();
    double get_max_x();
    double get_max_y();
    double get_exclusion_x();
    double get_exclusion_y();
    double get_ball_exclusion_radius();
    double get_ball_exclusion_height();
    double get_occupancy_threshold();
    bool get_enable_obstacle_avoidance();
    double get_freekick_start_placing_safe_distance();
    double get_freekick_start_placing_avoid_secs();
    double get_obstacle_memory_msecs();
    bool get_avoid_during_kick();
    double get_kick_ao_safe_dist();
    bool get_avoid_during_chase();
    double get_chase_ao_safe_dist();
    double get_collision_threshold();
    double get_safe_distance();
    double get_avoid_secs();


    int get_retry_max_count();

    string get_image_camera_info_topic();
    string get_depth_image_topic();
    string get_depth_camera_info_topic();

    // Phase1 extra strategy flags
    bool get_enable_shoot();

    // Phase1 parameter blocks (populated in Brain::loadConfig)
    Phase1BallPredictorConfig   ballPredictor;
    Phase1ChaseConfig           chase;
    Phase1KickConfig            kick;
    Phase1KickSelectorConfig    kickSelector;
    Phase1PowerShootConfig      powerShoot;
    Phase1CooperationConfig     cooperation;
    Phase1HeadControllerConfig  headController;
    Phase1TrainingLoggerConfig  trainingLogger;

    FieldDimensions fieldDimensions;
    vector<FieldLine> mapLines;
    vector<MapMarking> mapMarkings;


    int cameraImageWidth = 1280;
    int cameraImageHeight = 720;


    double depthCameraFovX = deg2rad(90);
    double depthCameraFovY = deg2rad(65);
    double depthCameraFx = 643.898;
    double depthCameraFy = 643.216;
    double depthCameraCx = 649.038;
    double depthCameraCy = 357.21;

    Eigen::Matrix4d camToHead;

    void calcMapLines();
    void calcMapMarkings();

    void handle();
    
    void print(ostream &os);

private:
    Brain* brain;
};