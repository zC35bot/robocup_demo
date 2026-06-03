#pragma once

/**
 * @file training_logger.h
 * @brief Phase1 brain-side training data collector (spec §9).
 *
 * Writes one TrainingFrame per Brain::tick() to disk when enabled
 * (config training_logger.enable, default false to avoid impacting matches).
 *
 * NOTE on format: the spec calls for .mcap (ROS2-native). To keep the build
 * self-contained and dependency-free, this logger persists a fully-specified,
 * append-only CSV with all TrainingFrame fields, one file per logger session,
 * named with a start timestamp and robot_id (per-match separation is achieved
 * by restarting the soccer process / starting a new session). A converter to
 * .mcap can be layered on top of this stable schema later.
 */

#include <string>
#include <fstream>
#include <cstdint>

#include "training_frame.h"

class TrainingLogger {
public:
    TrainingLogger() = default;
    ~TrainingLogger();

    // enable: master switch; robotId/robotName used in the filename; logDir base directory.
    void init(bool enable, int robotId, const std::string &robotName, uint64_t startUnixUs);

    bool enabled() const { return enable_; }

    // Append one frame (no-op if disabled).
    void log(const TrainingFrame &frame);

    void close();

private:
    void writeHeader();

    bool enable_ = false;
    bool headerWritten_ = false;
    std::ofstream ofs_;
    std::string filePath_;
    uint64_t frameCount_ = 0;
};
