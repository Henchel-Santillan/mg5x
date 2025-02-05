#pragma once

#include <gsl/span>
#include "state.hpp"

namespace mg5x {

class tile_generator {
public:
    constexpr explicit tile_generator(gsl::span<State> vertices)
        : vertices(vertices) {}
    
private:
    gsl::span<State> vertices;
};

}
