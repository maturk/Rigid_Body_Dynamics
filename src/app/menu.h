#include <string>
#include <vector>

namespace crl {
namespace app {
namespace rigidbody {

enum SimulationScene {
    Projectile = 0,
    FixedSprings = 1,
    RigidBodySprings = 2,
    Collision = 3, 
};

std::vector<std::string> simulationSceneNames = {
    "Projectile",    //
    "Fixed Springs",  //
    "Rigid Body Springs", //
    "Collision", //
};

enum SimulationEngine {
    Explicit = 0,
    Symplectic = 1
};

std::vector<std::string> simulationEngineNames = {
    "Explicit",    //
    "Symplectic",  //
};

}  // namespace rigidbody
}  // namespace app
}  // namespace crl