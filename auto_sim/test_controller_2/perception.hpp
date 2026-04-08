#pragma once

#include "types.hpp"
#include <vector>

inline std::vector<Cone> cones {};
inline void mock_perception(std::vector<Cone>&& new_cones) {
    cones = std::move(new_cones);
}