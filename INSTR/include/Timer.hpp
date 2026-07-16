// include/Timer.hpp
#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <cstdio>

// Accumulates per-scope timings and reports every N frames.
struct TimerStats {
    long long total_us = 0;
    long long count    = 0;
    long long max_us   = 0;
};

struct Timer {
    TimerStats& stats;
    std::chrono::steady_clock::time_point t0;

    explicit Timer(TimerStats& s)
        : stats(s), t0(std::chrono::steady_clock::now()) {}

    ~Timer() {
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t0).count();
        stats.total_us += us;
        stats.count++;
        if (us > stats.max_us) stats.max_us = us;
    }

};

inline void reportTimer(const char* name, TimerStats& s) {
        if (s.count == 0) return;
        fprintf(stderr, "  %-12s avg %6.2f ms   max %6.2f ms   n=%lld\n",
                name,
                (s.total_us / (double)s.count) / 1000.0,
                s.max_us / 1000.0,
                s.count);
        s.total_us = 0; s.count = 0; s.max_us = 0;  // reset window
    }

#endif