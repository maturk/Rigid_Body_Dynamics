//
// Created by Dongho Kang on 25.04.22.
//

#ifndef A5_RBEXPLICITENGINE_H
#define A5_RBEXPLICITENGINE_H

#include "sim/RBEngine.h"

namespace crl {

/**
 * simulation world implemented with explicit integration scheme.
 */
class RBExplicitEngine : public RBEngine {
public:
    RBExplicitEngine() : RBEngine() {}

    ~RBExplicitEngine() override = default;

    void step(double dt) override {
        // update external force and torque
        updateForceForGravity();
        updateForceAndTorqueForSprings();

        // update states of rbs (integration)
        for (uint i = 0; i < rbs.size(); i++) {
            RB *rb = rbs[i];
            // retrieve saved force and tau
            V3D f = f_ext[i];
            V3D tau = tau_ext[i];

            // TODO: Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing velocity.
            //
            // Hint:
            // - complete the function,
            // Quaternion updateRotationGivenAngularVelocity(const Quaternion &q, const V3D &angularVelocity, double dt)
            // in src/libs/sim/include/sim/RBEngine.h and use it for updating orientation of rigidbody.
            // - recall, you need to compute 3x3 moment of inertia matrix expressed in world frame.

            // Notes:
            // Explicit Euler integration is the following :
            // v_n+1 = v_n + dt * f_n(x_n) / m
            // x_n+1 = x_n + dt * v_n+1
            

            // Update velocity
            V3D v_n = rb->state.velocity;
            rb->state.velocity = rb->state.velocity + V3D(dt * f / rb->rbProps.mass);         // TODO: change this!

            // Update angular velocity
            // ang_vel = ang_vel + dt * I_IB.inverse() * (tau - cross(ang_vel, I_IB * ang_vel));
            // note I_IB = C_IB * I_B * C_BI.inverse()
            Matrix C_IB = rb->state.orientation.toRotationMatrix();
            Matrix I_B = rb->rbProps.MOI_local;
            Matrix I = C_IB * I_B * C_IB.inverse();

            V3D w_n = rb->state.angularVelocity;
            rb->state.angularVelocity = rb->state.angularVelocity + V3D(dt*I.inverse()*(tau - rb->state.angularVelocity.cross(V3D(I*rb->state.angularVelocity))));  // TODO: change this!

            if (simulateCollisions && rb->rbProps.collision)
                updateVelocityAfterCollision(rb);

            // TODO: Ex.1 Numerical Integration
            // implement forward (explicit) Euler integration scheme for computing pose.
            rb->state.pos = rb->state.pos + V3D(v_n) * dt;  // TODO: change this!
            rb->state.orientation = updateRotationGivenAngularVelocity(
                rb->state.orientation, w_n, dt);
        }

        // clean up
        for (uint i = 0; i < f_ext.size(); i++) {
            f_ext[i] = V3D();
            tau_ext[i] = V3D();
        }
    }
};

}  // namespace crl

#endif  //A5_RBEXPLICITENGINE_H
