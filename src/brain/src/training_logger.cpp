#include "training_logger.h"

#include <filesystem>
#include <sstream>
#include <iostream>

TrainingLogger::~TrainingLogger() {
    close();
}

void TrainingLogger::init(bool enable, int robotId, const std::string &robotName, uint64_t startUnixUs) {
    enable_ = enable;
    if (!enable_) return;

    std::string baseDir = "training_logs";
    try {
        std::filesystem::create_directories(baseDir);
    } catch (const std::exception &e) {
        std::cerr << "[TrainingLogger] failed to create dir " << baseDir << ": " << e.what() << std::endl;
        enable_ = false;
        return;
    }

    std::ostringstream name;
    name << baseDir << "/training_"
         << "robot" << robotId;
    if (!robotName.empty()) name << "_" << robotName;
    name << "_" << startUnixUs << ".csv";
    filePath_ = name.str();

    ofs_.open(filePath_, std::ios::out | std::ios::trunc);
    if (!ofs_.is_open()) {
        std::cerr << "[TrainingLogger] failed to open " << filePath_ << std::endl;
        enable_ = false;
        return;
    }
    writeHeader();
    std::cout << "[TrainingLogger] logging to " << filePath_ << std::endl;
}

void TrainingLogger::writeHeader() {
    if (headerWritten_) return;
    ofs_ << "timestamp_us,"
         << "raw_ball_robot_x,raw_ball_robot_y,raw_ball_conf,"
         << "bbox_x,bbox_y,bbox_w,bbox_h,"
         << "filtered_ball_field_x,filtered_ball_field_y,"
         << "pred100_field_x,pred100_field_y,pred300_field_x,pred300_field_y,"
         << "pred300_valid,using_field_frame,"
         << "pred100_robot_x,pred100_robot_y,"
         << "mode_prob_stationary,mode_prob_rolling,ball_confidence,"
         << "robot_pose_x,robot_pose_y,robot_pose_theta,"
         << "robot_vel_x,robot_vel_y,robot_vel_theta,"
         << "head_yaw,head_pitch,"
         << "imu_acc_x,imu_acc_y,imu_acc_z,"
         << "player_role,decision,is_lead,cost,"
         << "kick_result,abort_reason,"
         << "tm_age_ms_1,tm_age_ms_2,tm_age_ms_3,tm_age_ms_4,"
         << "fall_state,game_state"
         << "\n";
    headerWritten_ = true;
}

void TrainingLogger::log(const TrainingFrame &f) {
    if (!enable_ || !ofs_.is_open()) return;

    ofs_ << f.timestamp_us << ','
         << f.raw_ball_robot[0] << ',' << f.raw_ball_robot[1] << ',' << f.raw_ball_conf << ','
         << f.bbox_xywh[0] << ',' << f.bbox_xywh[1] << ',' << f.bbox_xywh[2] << ',' << f.bbox_xywh[3] << ','
         << f.filtered_ball_field[0] << ',' << f.filtered_ball_field[1] << ','
         << f.pred100_field[0] << ',' << f.pred100_field[1] << ','
         << f.pred300_field[0] << ',' << f.pred300_field[1] << ','
         << static_cast<int>(f.pred300_valid) << ',' << static_cast<int>(f.using_field_frame) << ','
         << f.pred100_robot[0] << ',' << f.pred100_robot[1] << ','
         << f.mode_prob[0] << ',' << f.mode_prob[1] << ',' << f.ball_confidence << ','
         << f.robot_pose[0] << ',' << f.robot_pose[1] << ',' << f.robot_pose[2] << ','
         << f.robot_vel[0] << ',' << f.robot_vel[1] << ',' << f.robot_vel[2] << ','
         << f.head_pose[0] << ',' << f.head_pose[1] << ','
         << f.imu_acc[0] << ',' << f.imu_acc[1] << ',' << f.imu_acc[2] << ','
         << static_cast<int>(f.player_role) << ',' << static_cast<int>(f.decision) << ','
         << static_cast<int>(f.is_lead) << ',' << f.cost << ','
         << static_cast<int>(f.kick_result) << ',' << static_cast<int>(f.abort_reason) << ','
         << f.tm_age_ms[0] << ',' << f.tm_age_ms[1] << ',' << f.tm_age_ms[2] << ',' << f.tm_age_ms[3] << ','
         << static_cast<int>(f.fall_state) << ',' << static_cast<int>(f.game_state)
         << '\n';

    frameCount_++;
    // Flush periodically so data survives an unclean shutdown without hurting throughput.
    if ((frameCount_ % 50) == 0) ofs_.flush();
}

void TrainingLogger::close() {
    if (ofs_.is_open()) {
        ofs_.flush();
        ofs_.close();
    }
}
