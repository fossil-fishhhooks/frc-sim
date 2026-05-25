#pragma once
#include "core/MotorModel.h"
#include <string>
#include <unordered_map>
#include <optional>
#include <vector>

//Lookup motor models

class MotorRegistry {
public:
    MotorRegistry();   // pre-populates with builtins

    // Parse all *.json files in dir and add/override entries.
    // Silently skips unreadable files (logs a warning).
    void LoadFromDirectory(const std::string& dir);

    // Returns pointer to profile, nullptr if name not found.
    const MotorModel* Lookup(const std::string& name) const;

    // Returns all registered names (for debug / listing).
    std::vector<std::string> Names() const;

private:
    std::unordered_map<std::string, MotorModel> m_profiles;
};
