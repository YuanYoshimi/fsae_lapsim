#include "lapsim/Vehicle.hpp"

#include <gtest/gtest.h>

TEST(VehicleTest, DefaultFromYamlHasPositiveMass) {
    auto vehicle = lapsim::Vehicle::from_yaml("");
    EXPECT_GT(vehicle.mass_kg, 0.0);
}

TEST(VehicleTest, DefaultFromYamlHasPositiveWheelbase) {
    auto vehicle = lapsim::Vehicle::from_yaml("");
    EXPECT_GT(vehicle.wheelbase_m, 0.0);
}
