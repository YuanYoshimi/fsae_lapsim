#include "lapsim/TrackLoader.hpp"

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace lapsim {

auto TrackLoader::load(const std::string& path) -> Track {
    YAML::Node root = YAML::LoadFile(path);
    const auto& track_node = root["track"];

    Track track;
    track.set_name(track_node["name"].as<std::string>("Unnamed Track"));
    track.set_closed(track_node["closed"].as<bool>(false));

    if (track_node["width_m"]) {
        track.set_width(track_node["width_m"].as<double>());
    } else {
        std::cerr << "Warning: track YAML missing 'width_m', defaulting to 4.0 m\n";
    }

    auto start_seq = track_node["start_point"];
    Vec2 cursor_pt{start_seq[0].as<double>(), start_seq[1].as<double>()};
    double cursor_hdg = track_node["start_heading"].as<double>();

    for (const auto& seg_node : track_node["segments"]) {
        std::string id   = seg_node["id"].as<std::string>();
        std::string type = seg_node["type"].as<std::string>();

        if (type == "straight") {
            double len = seg_node["length"].as<double>();
            auto seg = std::make_unique<Straight>(id, cursor_pt, cursor_hdg, len);
            cursor_pt  = seg->end_point();
            cursor_hdg = seg->end_heading();
            track.add_segment(std::move(seg));

        } else if (type == "arc") {
            double radius = seg_node["radius"].as<double>();
            double swept  = seg_node["swept_angle"].as<double>();
            auto seg = std::make_unique<Arc>(id, cursor_pt, cursor_hdg, radius, swept);
            cursor_pt  = seg->end_point();
            cursor_hdg = seg->end_heading();
            track.add_segment(std::move(seg));

        } else {
            std::cerr << "TrackLoader: unknown segment type '" << type
                      << "' for segment " << id << '\n';
        }
    }

    auto result = track.validate();
    if (!result.is_valid) {
        std::cerr << "Track validation FAILED:\n";
        for (const auto& err : result.errors) {
            std::cerr << "  - " << err << '\n';
        }
    }

    return track;
}

} // namespace lapsim
