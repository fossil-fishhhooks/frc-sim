#pragma once
#include <cstdio>
#include <cstdarg>
#include <chrono>
#include <mutex>


namespace logger {

    namespace detail {
        // Statics inside inline functions = one instance across all TUs, no ODR
        inline std::chrono::system_clock::time_point& start_time() {
            static auto t = std::chrono::system_clock::now();
            return t;
        }
        inline std::mutex& log_mutex() {
            static std::mutex m;
            return m;
        }
        inline double elapsed_ms() {
            return std::chrono::duration<double, std::milli>(
                std::chrono::system_clock::now() - start_time()
            ).count();
        }
        inline void write(const char* level, const char* msg, va_list args) {
            std::lock_guard<std::mutex> lock(log_mutex());
            printf("%011.2f | %s | ", elapsed_ms(), level);
            vprintf(msg, args);
            printf("\n");
        }
    }

    // Call once at the start of main() to reset the timer to zero.
    // Safe to skip — timer starts at first log call otherwise.
    inline void init() {
        detail::start_time() = std::chrono::system_clock::now();
    }

    inline void debug(const char* msg, ...) {
        va_list args; va_start(args, msg);
        detail::write("D", msg, args);
        va_end(args);
    }
    inline void info(const char* msg, ...) {
        va_list args; va_start(args, msg);
        detail::write("I", msg, args);
        va_end(args);
    }
    inline void warning(const char* msg, ...) {
        va_list args; va_start(args, msg);
        detail::write("W", msg, args);
        va_end(args);
    }
    inline void error(const char* msg, ...) {
        va_list args; va_start(args, msg);
        detail::write("E", msg, args);
        va_end(args);
    }
    inline void wtf(const char* msg, ...) {
        va_list args; va_start(args, msg);
        detail::write("WTF", msg, args);
        va_end(args);
    }
    inline float elapsed() {
        return detail::elapsed_ms();
    }

} // namespace logger

// ── Macro layer ───────────────────────────────────────────────────────────────
// Identical to the functions but LOG_DEBUG compiles away in release.

#define LOG_INFO(...)    logger::info(__VA_ARGS__)
#define LOG_WARN(...)    logger::warning(__VA_ARGS__)
#define LOG_ERROR(...)   logger::error(__VA_ARGS__)
#define LOG_WTF(...)     logger::wtf(__VA_ARGS__)

#ifdef NDEBUG
#define LOG_DEBUG(...) do {} while(0)
#else
#define LOG_DEBUG(...) logger::debug(__VA_ARGS__)
#endif
