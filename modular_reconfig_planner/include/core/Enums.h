#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace mrr {

enum class ModuleStatus {
    Free,
    Connected,
    Moving,
    Blocked
};

enum class EnvironmentType {
    Flat,
    NarrowPassage,
    RoughSurface,
    Step,
    Gap,
    TargetZone,
    Unknown
};

inline std::string normalizeToken(std::string s) {
    std::string out;
    out.reserve(s.size());
    for (char c : s) {
        if (c == '-' || c == ' ') out.push_back('_');
        else out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    return out;
}

inline std::string toString(ModuleStatus s) {
    switch (s) {
        case ModuleStatus::Free: return "FREE";
        case ModuleStatus::Connected: return "CONNECTED";
        case ModuleStatus::Moving: return "MOVING";
        case ModuleStatus::Blocked: return "BLOCKED";
    }
    return "UNKNOWN";
}

inline std::string toString(EnvironmentType t) {
    switch (t) {
        case EnvironmentType::Flat: return "FLAT";
        case EnvironmentType::NarrowPassage: return "NARROW_PASSAGE";
        case EnvironmentType::RoughSurface: return "ROUGH_SURFACE";
        case EnvironmentType::Step: return "STEP";
        case EnvironmentType::Gap: return "GAP";
        case EnvironmentType::TargetZone: return "TARGET_ZONE";
        default: return "UNKNOWN";
    }
}

inline EnvironmentType environmentTypeFromString(const std::string& value) {
    const std::string s = normalizeToken(value);
    if (s == "FLAT") return EnvironmentType::Flat;
    if (s == "NARROW_PASSAGE" || s == "NARROWPASSAGE") return EnvironmentType::NarrowPassage;
    if (s == "ROUGH_SURFACE" || s == "ROUGHSURFACE") return EnvironmentType::RoughSurface;
    if (s == "STEP") return EnvironmentType::Step;
    if (s == "GAP") return EnvironmentType::Gap;
    if (s == "TARGET_ZONE" || s == "TARGETZONE") return EnvironmentType::TargetZone;
    return EnvironmentType::Unknown;
}

inline std::string preferredConfigurationFor(EnvironmentType t) {
    switch (t) {
        case EnvironmentType::Flat: return "WIDE_STABLE";
        case EnvironmentType::NarrowPassage: return "LINE";
        case EnvironmentType::RoughSurface: return "SNAKE";
        case EnvironmentType::Step: return "CLIMB";
        case EnvironmentType::Gap: return "BRIDGE";
        case EnvironmentType::TargetZone: return "MANIPULATOR";
        default: return "WIDE_STABLE";
    }
}

} // namespace mrr
