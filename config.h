#include <fstream>
#include <json.hpp>
#include "memory"
#include "log.h"

using json = nlohmann::json;

#ifndef CONFIG_H
#define CONFIG_H

namespace Core {
    class Config {
        public:
            static void Init();
            inline static json GetCoreConfig() {
                return *CoreConfig;
            }
        private:
            static std::shared_ptr<json> CoreConfig;
    };
}

#define CONFIG_VAR(...) ::Core::Config::GetCoreConfig()[__VA_ARGS__]

#endif