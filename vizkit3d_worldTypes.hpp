#ifndef vizkit3d_world_TYPES_HPP
#define vizkit3d_world_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <iostream>
#include <base/samples/Joints.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace vizkit3d_world {

    struct ModelState {
        std::string model_name;
        base::samples::RigidBodyState pose;
        base::samples::Joints joints;
    };
}

#endif

