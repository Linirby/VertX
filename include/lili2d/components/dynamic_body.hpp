#pragma once

#include "lili2d/geometry/vec2.hpp"
#include <type_traits>
namespace lili {

/// @brief Physics component that reprensent a Dynamic Body
struct cDynamicBody
{
    Vec2 force = {};               ///< Force accumulator
    Vec2 impulse = {};             ///< Impulse accumulator
    Vec2 damping = { 0.0f, 0.0f }; ///< Damping on both axes
    float bounciness = 0.0f;       ///< Restitution (0: no bounce, 1: elastic)
    float mass = 1.0f;             ///< Basically the mass :3
    bool grounded = false;         ///< Flags that says when on the ground
};

static_assert(
    std::is_trivially_copyable_v<cDynamicBody>,
    "cDynamicBody component must be trivially copyable"
);

} // namespace lili
