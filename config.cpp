#include "config.h"

namespace Core {
    std::shared_ptr<json> Config::CoreConfig;

    void Config::Init() {
        std::ifstream f("../config.json");
        if(!f.is_open()) {
            //CORE_ERROR("Erro ao abrir o arquivo de configuracao");
            exit(0);
        }
        CoreConfig = std::make_shared<json>(json::parse(f));
    }
}