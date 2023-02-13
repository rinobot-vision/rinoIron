#include "memory"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#ifndef LOG_H
#define LOG_H

namespace Core {
    class Log {
        public:
            static void Init();
            inline static std::shared_ptr<spdlog::logger>& GetCoreLoger() {
                return CoreLogger;
            }
        private:
            static std::shared_ptr<spdlog::logger> CoreLogger;
    };
}

#endif

#define CORE_ERROR(...) ::Core::Log::GetCoreLoger()->error(__VA_ARGS__)
#define CORE_TRACE(...) ::Core::Log::GetCoreLoger()->trace(__VA_ARGS__)
#define CORE_WARN(...) ::Core::Log::GetCoreLoger()->warn(__VA_ARGS__)
#define CORE_INFO(...) ::Core::Log::GetCoreLoger()->info(__VA_ARGS__)