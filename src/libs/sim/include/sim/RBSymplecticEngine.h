//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBSYMPLECTICENGINE_H
#define A5_RBSYMPLECTICENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with symplectic integration scheme.
 */
class RBSymplecticEngine : public RBEngine {
public:
    RBSymplecticEngine() : RBEngine() {}

    ~RBSymplecticEngine() override = default;

    void step(double dt) override {
        // external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.3 Stable Simulation
            // why our simulation is blown up? let's make it more stable using
            // symplectic Euler integration!

            // Velocity and Angular velocity update
            rb->state.velocity = rb->state.velocity + V3D(dt * f / rb->rbProps.mass);  // Velocity is same as before in Explicit Euler scheme

            Matrix C_IB = rb->state.orientation.toRotationMatrix();
            Matrix I_B = rb->rbProps.MOI_local;
            Matrix I = C_IB * I_B * C_IB.inverse();
            rb->state.angularVelocity = rb->state.angularVelocity + V3D(dt*I.inverse()*(tau - rb->state.angularVelocity.cross(V3D(I*rb->state.angularVelocity)))); // Angular velocity is same as before in Explicit Euler scheme

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Ex.3 Stable Simulation
            // implement forward (explicit) Euler integration scheme for computing pose.

            // Notes:
            // Symplectic Euler integration is the following :
            // x_n+1 = x_n + dt * v_n
            // v_n+1 = v_n + dt * f_n(x_n+1) / m
            // i.e use x_n+1 that was calculated to update velocity at n+1 (instead of x_n used in explicit Euler integration) 
            V3D v_nplus1 = rb->state.velocity ; // Saving velocity at n+1 after computing new velocity at n
            V3D w_nplus1 = rb->state.angularVelocity ; // Saving angular velocity at n+1

            rb->state.pos = rb->state.pos + v_nplus1 * dt;  // TODO: change this!
            rb->state.orientation = updateRotationGivenAngularVelocity(
                rb->state.orientation, w_nplus1, dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBSYMPLECTICENGINE_H
