#pragma once

#include <utils/geoms.h>
#include <utils/mathUtils.h>

namespace crl {

class RBGlobals {
public:
    // gravitational acceleration
    static double g;
    // this is the direction of the up-vector
    static V3D worldUp;
    // and the ground plane
    static Plane groundPlane;
};



}  // namespace crl