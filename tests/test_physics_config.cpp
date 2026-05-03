#include "lapsim/PhysicsConfig.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

using namespace lapsim;

// ── Preset flag values ────────────────────────────────────────────────

TEST(PhysicsConfigTest, BasicPresetFlags) {
    auto cfg = PhysicsConfig::basic();
    EXPECT_FALSE(cfg.continuous_profile);
    EXPECT_FALSE(cfg.friction_circle);
    EXPECT_FALSE(cfg.aero);
    EXPECT_DOUBLE_EQ(cfg.ds, 0.5);
}

TEST(PhysicsConfigTest, QssPresetFlags) {
    auto cfg = PhysicsConfig::qss();
    EXPECT_TRUE(cfg.continuous_profile);
    EXPECT_FALSE(cfg.friction_circle);
    EXPECT_FALSE(cfg.aero);
    EXPECT_DOUBLE_EQ(cfg.ds, 0.5);
}

TEST(PhysicsConfigTest, FcPresetFlags) {
    auto cfg = PhysicsConfig::fc();
    EXPECT_TRUE(cfg.continuous_profile);
    EXPECT_TRUE(cfg.friction_circle);
    EXPECT_FALSE(cfg.aero);
    EXPECT_DOUBLE_EQ(cfg.ds, 0.5);
}

TEST(PhysicsConfigTest, AeroPresetFlags) {
    auto cfg = PhysicsConfig::aero_preset();
    EXPECT_TRUE(cfg.continuous_profile);
    EXPECT_TRUE(cfg.friction_circle);
    EXPECT_TRUE(cfg.aero);
    EXPECT_DOUBLE_EQ(cfg.ds, 0.5);
}

// ── from_preset roundtrip ─────────────────────────────────────────────

TEST(PhysicsConfigTest, FromPresetBasic) {
    auto a = PhysicsConfig::basic();
    auto b = PhysicsConfig::from_preset("basic");
    EXPECT_EQ(a.continuous_profile, b.continuous_profile);
    EXPECT_EQ(a.friction_circle,    b.friction_circle);
    EXPECT_EQ(a.aero,               b.aero);
    EXPECT_DOUBLE_EQ(a.ds, b.ds);
}

TEST(PhysicsConfigTest, FromPresetQss) {
    auto a = PhysicsConfig::qss();
    auto b = PhysicsConfig::from_preset("qss");
    EXPECT_EQ(a.continuous_profile, b.continuous_profile);
    EXPECT_EQ(a.friction_circle,    b.friction_circle);
    EXPECT_EQ(a.aero,               b.aero);
}

TEST(PhysicsConfigTest, FromPresetFc) {
    auto a = PhysicsConfig::fc();
    auto b = PhysicsConfig::from_preset("fc");
    EXPECT_EQ(a.continuous_profile, b.continuous_profile);
    EXPECT_EQ(a.friction_circle,    b.friction_circle);
    EXPECT_EQ(a.aero,               b.aero);
}

TEST(PhysicsConfigTest, FromPresetAero) {
    auto a = PhysicsConfig::aero_preset();
    auto b = PhysicsConfig::from_preset("aero");
    EXPECT_EQ(a.continuous_profile, b.continuous_profile);
    EXPECT_EQ(a.friction_circle,    b.friction_circle);
    EXPECT_EQ(a.aero,               b.aero);
}

TEST(PhysicsConfigTest, FromPresetThrowsOnGarbage) {
    EXPECT_THROW(PhysicsConfig::from_preset("garbage"), std::invalid_argument);
    EXPECT_THROW(PhysicsConfig::from_preset(""),        std::invalid_argument);
    EXPECT_THROW(PhysicsConfig::from_preset("BASIC"),   std::invalid_argument);
}

// ── preset_name roundtrip ─────────────────────────────────────────────

TEST(PhysicsConfigTest, PresetNameRoundtrip) {
    EXPECT_EQ(PhysicsConfig::basic().preset_name(),       "basic");
    EXPECT_EQ(PhysicsConfig::qss().preset_name(),         "qss");
    EXPECT_EQ(PhysicsConfig::fc().preset_name(),          "fc");
    EXPECT_EQ(PhysicsConfig::aero_preset().preset_name(), "aero");
}

TEST(PhysicsConfigTest, PresetNameCustom) {
    // continuous_profile off but aero on — not a standard preset
    PhysicsConfig cfg{.continuous_profile = false,
                      .friction_circle    = false,
                      .aero               = true};
    EXPECT_EQ(cfg.preset_name(), "custom");
}

TEST(PhysicsConfigTest, PresetNameIgnoresDs) {
    auto cfg = PhysicsConfig::qss();
    cfg.ds = 0.25;
    EXPECT_EQ(cfg.preset_name(), "qss");
}

// ── description ───────────────────────────────────────────────────────

TEST(PhysicsConfigTest, DescriptionBasic) {
    auto desc = PhysicsConfig::basic().description();
    EXPECT_NE(desc.find("basic"), std::string::npos);
}

TEST(PhysicsConfigTest, DescriptionWarnsOnInvalidCombo) {
    PhysicsConfig cfg{.continuous_profile = false,
                      .friction_circle    = true,
                      .aero               = false};
    auto desc = cfg.description();
    EXPECT_NE(desc.find("WARNING"), std::string::npos);
}
