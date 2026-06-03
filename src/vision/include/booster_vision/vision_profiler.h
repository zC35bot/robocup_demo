#pragma once

/**
 * @file vision_profiler.h
 * @brief Phase1 lightweight vision pipeline profiler (spec §8).
 *
 * Pure instrumentation: it records per-frame stage durations and reports
 * p50/p95/fps/drop/jitter every N frames. It does NOT touch detection/publish
 * logic. All timing uses std::chrono::steady_clock (monotonic).
 *
 * Stages (ms): receive, preprocess, inference, postprocess (decode + pose),
 * publish; plus end-to-end (e2e = publish_end - frame_start).
 */

#include <chrono>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <cstdint>

namespace booster_vision {

class VisionProfiler {
public:
    using clock = std::chrono::steady_clock;

    struct Config {
        double alert_fps_min = 40.0;
        double alert_drop_rate_max = 2.0;     // percent
        double alert_e2e_p95_ms_max = 20.0;
        double alert_jitter_ms_max = 5.0;
        int    report_every_n_frames = 100;
        bool   enabled = true;
    };

    void configure(const Config &cfg) { cfg_ = cfg; }
    bool enabled() const { return cfg_.enabled; }

    // -------- per-frame instrumentation --------
    void frameStart() {
        t_start_ = clock::now();
        t_recv_ = t_start_;
    }
    void markReceiveDone() { t_recv_ = clock::now(); }
    void markPreprocessDone() { t_pre_ = clock::now(); }
    void markInferenceDone() { t_inf_ = clock::now(); }
    void markPostprocessDone() { t_post_ = clock::now(); }
    void markPublishDone() {
        auto t_pub = clock::now();

        double recv = ms(t_start_, t_recv_);
        double pre = ms(t_recv_, t_pre_);
        double inf = ms(t_pre_, t_inf_);
        double post = ms(t_inf_, t_post_);
        double pub = ms(t_post_, t_pub);
        double e2e = ms(t_start_, t_pub);

        recv_.push_back(recv);
        pre_.push_back(pre);
        inf_.push_back(inf);
        post_.push_back(post);
        pub_.push_back(pub);
        e2e_.push_back(e2e);

        if (haveLastFrame_) {
            interval_.push_back(ms(lastFrameStart_, t_start_));
        }
        lastFrameStart_ = t_start_;
        haveLastFrame_ = true;

        if (++frameCount_ >= cfg_.report_every_n_frames) {
            report();
            resetWindow();
        }
    }

private:
    static double ms(const clock::time_point &a, const clock::time_point &b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    }

    static double percentile(std::vector<double> v, double p) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        double idx = p / 100.0 * (v.size() - 1);
        size_t lo = static_cast<size_t>(std::floor(idx));
        size_t hi = static_cast<size_t>(std::ceil(idx));
        if (hi >= v.size()) hi = v.size() - 1;
        double frac = idx - lo;
        return v[lo] * (1.0 - frac) + v[hi] * frac;
    }

    static double median(std::vector<double> v) { return percentile(std::move(v), 50.0); }

    void report() {
        // fps + jitter from inter-frame intervals
        double fps = 0.0, jitter = 0.0, dropRate = 0.0;
        if (!interval_.empty()) {
            double meanInt = 0.0;
            for (double x : interval_) meanInt += x;
            meanInt /= interval_.size();
            if (meanInt > 1e-6) fps = 1000.0 / meanInt;

            double var = 0.0;
            for (double x : interval_) var += (x - meanInt) * (x - meanInt);
            jitter = std::sqrt(var / interval_.size());

            // drop estimate: intervals notably longer than the median => likely dropped frames
            double medInt = median(interval_);
            int drops = 0;
            for (double x : interval_) {
                if (medInt > 1e-6 && x > 1.8 * medInt) drops++;
            }
            dropRate = 100.0 * static_cast<double>(drops) / static_cast<double>(interval_.size());
        }

        double infP50 = percentile(inf_, 50.0);
        double infP95 = percentile(inf_, 95.0);
        double e2eP50 = percentile(e2e_, 50.0);
        double e2eP95 = percentile(e2e_, 95.0);

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "[VisionProfiler] fps=" << fps << " drop=" << std::setprecision(1) << dropRate << "%\n"
                  << "  receive    p50=" << percentile(recv_, 50.0) << "ms p95=" << percentile(recv_, 95.0) << "ms\n"
                  << "  preprocess p50=" << percentile(pre_, 50.0) << "ms p95=" << percentile(pre_, 95.0) << "ms\n"
                  << "  inference  p50=" << infP50 << "ms p95=" << infP95 << "ms\n"
                  << "  postproc   p50=" << percentile(post_, 50.0) << "ms p95=" << percentile(post_, 95.0) << "ms\n"
                  << "  publish    p50=" << percentile(pub_, 50.0) << "ms p95=" << percentile(pub_, 95.0) << "ms\n"
                  << "  e2e        p50=" << e2eP50 << "ms p95=" << e2eP95 << "ms  jitter=" << jitter << "ms"
                  << std::endl;

        // alerts
        if (fps < cfg_.alert_fps_min)
            std::cerr << "[VisionProfiler][ALERT] fps " << fps << " < " << cfg_.alert_fps_min << std::endl;
        if (dropRate > cfg_.alert_drop_rate_max)
            std::cerr << "[VisionProfiler][ALERT] drop " << dropRate << "% > " << cfg_.alert_drop_rate_max << "%" << std::endl;
        if (e2eP95 > cfg_.alert_e2e_p95_ms_max)
            std::cerr << "[VisionProfiler][ALERT] e2e p95 " << e2eP95 << "ms > " << cfg_.alert_e2e_p95_ms_max << "ms" << std::endl;
        if (jitter > cfg_.alert_jitter_ms_max)
            std::cerr << "[VisionProfiler][ALERT] jitter " << jitter << "ms > " << cfg_.alert_jitter_ms_max << "ms" << std::endl;
    }

    void resetWindow() {
        frameCount_ = 0;
        recv_.clear();
        pre_.clear();
        inf_.clear();
        post_.clear();
        pub_.clear();
        e2e_.clear();
        interval_.clear();
    }

    Config cfg_;
    int frameCount_ = 0;
    bool haveLastFrame_ = false;
    clock::time_point t_start_, t_recv_, t_pre_, t_inf_, t_post_, lastFrameStart_;
    std::vector<double> recv_, pre_, inf_, post_, pub_, e2e_, interval_;
};

} // namespace booster_vision
