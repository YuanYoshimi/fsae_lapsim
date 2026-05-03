#include "lapsim/PhysicsConfig.hpp"

#include <sstream>
#include <stdexcept>

namespace lapsim {

auto PhysicsConfig::basic() -> PhysicsConfig {
    return {.continuous_profile = false,
            .friction_circle    = false,
            .aero               = false,
            .ds                 = 0.5};
}

auto PhysicsConfig::qss() -> PhysicsConfig {
    return {.continuous_profile = true,
            .friction_circle    = false,
            .aero               = false,
            .ds                 = 0.5};
}

auto PhysicsConfig::fc() -> PhysicsConfig {
    return {.continuous_profile = true,
            .friction_circle    = true,
            .aero               = false,
            .ds                 = 0.5};
}

auto PhysicsConfig::aero_preset() -> PhysicsConfig {
    return {.continuous_profile = true,
            .friction_circle    = true,
            .aero               = true,
            .ds                 = 0.5};
}

auto PhysicsConfig::from_preset(const std::string& name) -> PhysicsConfig {
    if (name == "basic") return basic();
    if (name == "qss")   return qss();
    if (name == "fc")    return fc();
    if (name == "aero")  return aero_preset();
    throw std::invalid_argument("Unknown preset: " + name);
}

auto PhysicsConfig::preset_name() const -> std::string {
    // ds value is intentionally ignored when matching presets;
    // a user may override ds without creating a "custom" config.
    if (!continuous_profile && !friction_circle && !aero) return "basic";
    if ( continuous_profile && !friction_circle && !aero) return "qss";
    if ( continuous_profile &&  friction_circle && !aero) return "fc";
    if ( continuous_profile &&  friction_circle &&  aero) return "aero";
    return "custom";
}

auto PhysicsConfig::description() const -> std::string {
    std::ostringstream oss;
    oss << preset_name() << ": ";

    if (!continuous_profile) {
        oss << "per-segment kinematics (basic)";
        if (friction_circle || aero) {
            oss << " [WARNING: FC and aero require continuous_profile]";
        }
        return oss.str();
    }

    oss << "QSS (ds=" << ds << " m)";
    if (friction_circle) oss << " + friction circle";
    if (aero)            oss << " + aero (drag+downforce)";
    return oss.str();
}

} // namespace lapsim
