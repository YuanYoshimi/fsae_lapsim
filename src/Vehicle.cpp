#include "lapsim/Vehicle.hpp"

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace lapsim {

auto Vehicle::from_yaml(const std::string& path) -> Vehicle {
    Vehicle v;

    // Sensible defaults (used when YAML is missing or path is empty).
    v.mass_kg        = 250.0;
    v.wheelbase_m    = 1.55;
    v.cda_m2         = 0.45;
    v.cla_m2         = 1.60;
    v.mu             = 0.7;
    v.max_accel_mps2 = 14.0;
    v.max_power_w    = 60'000.0;
    v.h_cg_m         = 0.27;
    v.g_mps2         = 9.81;
    v.aero           = Aero(1.225, 1.0, 1.5);

    if (path.empty()) return v;

    try {
        YAML::Node root = YAML::LoadFile(path);
        const auto& vn = root["vehicle"];
        if (vn) {
            v.mass_kg        = vn["mass_kg"].as<double>(v.mass_kg);
            v.wheelbase_m    = vn["wheelbase_m"].as<double>(v.wheelbase_m);
            v.h_cg_m         = vn["h_cg_m"].as<double>(v.h_cg_m);
            v.g_mps2         = vn["g"].as<double>(v.g_mps2);
            v.mu             = vn["mu"].as<double>(v.mu);
            v.max_accel_mps2 = vn["max_accel"].as<double>(v.max_accel_mps2);
            v.max_power_w    = vn["max_power_w"].as<double>(v.max_power_w);
            v.cda_m2         = vn["cda_m2"].as<double>(v.cda_m2);
            v.cla_m2         = vn["cla_m2"].as<double>(v.cla_m2);
        }

        const auto& an = root["aero"];
        if (an) {
            double rho = an["rho"].as<double>(1.225);
            double CdA = an["CdA"].as<double>(1.0);
            double ClA = an["ClA"].as<double>(1.5);
            v.aero = Aero(rho, CdA, ClA);
        }
    } catch (const std::exception& e) {
        std::cerr << "Vehicle::from_yaml: " << e.what()
                  << " — using defaults\n";
    }

    return v;
}

} // namespace lapsim
