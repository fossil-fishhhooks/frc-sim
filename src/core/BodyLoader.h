#pragma once
#include "core/BodyDef.h"
#include "core/MotorRegistry.h"
#include <string>
#include <optional>

// Loads one body JSON file
std::optional<BodyDef> LoadBodyDef(const std::string& json_path, const MotorRegistry& motors);
