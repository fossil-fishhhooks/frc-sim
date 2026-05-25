#include "core/MotorRegistry.h"
#include "io/EasyLog.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <filesystem>

namespace fs = std::filesystem;
using json   = nlohmann::json;


//Defaults
MotorRegistry::MotorRegistry()
{
    m_profiles["Kraken"]     = Motors::Kraken;
    m_profiles["Falcon500"]  = Motors::Falcon500;
    m_profiles["NEO"]        = Motors::NEO;
    m_profiles["NEO550"]     = Motors::NEO550;
    m_profiles["NEO_Vortex"] = Motors::NEO_Vortex;
    m_profiles["CIM"]        = Motors::CIM;
    m_profiles["MiniCIM"]    = Motors::MiniCIM;
    m_profiles["NEO845"]     = Motors::NEO_845;
}

void MotorRegistry::LoadFromDirectory(const std::string& dir)
{
    if (!fs::exists(dir)) {
        LOG_WARN("MotorRegistry: directory not found: %s (using builtins only)", dir.c_str());
        return;
    }

    int loaded = 0;
    for (auto& entry : fs::directory_iterator(dir)) {
        if (entry.path().extension() != ".json") continue;

        std::string path = entry.path().string();
        std::ifstream f(path);
        if (!f.is_open()) {
            LOG_WARN("MotorRegistry: cannot open %s", path.c_str());
            continue;
        }

        json j;
        try { f >> j; }
        catch (const json::parse_error& e) {
            LOG_WARN("MotorRegistry: parse error in %s: %s", path.c_str(), e.what());
            continue;
        }

        if (!j.contains("name")) {
            LOG_WARN("MotorRegistry: missing 'name' in %s", path.c_str());
            continue;
        }

        MotorModel m;
        m.stall_torque  = j.value("stall_torque",     0.0f);
        m.free_speed    = j.value("free_speed_rad_s",  0.0f);
        m.stall_current = j.value("stall_current",    0.0f);
        m.free_current  = j.value("free_current",     0.0f);

        std::string name = j["name"].get<std::string>();
        m_profiles[name] = m;
        LOG_DEBUG("MotorRegistry: loaded '%s' from %s", name.c_str(), path.c_str());
        ++loaded;
    }

    LOG_INFO("MotorRegistry: loaded %d motor(s) from %s", loaded, dir.c_str());
}


//Lookup
const MotorModel* MotorRegistry::Lookup(const std::string& name) const
{
    auto it = m_profiles.find(name);
    if (it == m_profiles.end()) {
        LOG_WARN("MotorRegistry: unknown motor profile '%s'", name.c_str());
        return nullptr;
    }
    return &it->second;
}

std::vector<std::string> MotorRegistry::Names() const
{
    std::vector<std::string> names;
    names.reserve(m_profiles.size());
    for (auto& [k, v] : m_profiles) names.push_back(k);
    return names;
}
