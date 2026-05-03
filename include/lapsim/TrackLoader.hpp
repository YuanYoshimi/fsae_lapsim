#pragma once

#include "lapsim/Track.hpp"

#include <string>

namespace lapsim {

/// Loads a Track from a YAML definition file.
///
/// Each segment is chain-constructed from the previous segment's
/// end point and heading; the YAML only specifies segment-local
/// parameters (type, length or radius+swept_angle).
class TrackLoader {
public:
    /// Load and validate a track from a YAML file.
    /// @param path Filesystem path to the track YAML.
    /// @return Fully constructed Track. Validation errors are printed to
    ///         stderr but the Track is still returned.
    [[nodiscard]] static auto load(const std::string& path) -> Track;
};

} // namespace lapsim
