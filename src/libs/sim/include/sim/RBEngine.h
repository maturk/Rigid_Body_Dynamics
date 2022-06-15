#pragma once

#include <sim/RB.h>
#include <sim/RBRenderer.h>
#include <sim/RBSpring.h>
#include <sim/RBUtils.h>

namespace crl {

// subfunctions
Quaternion updateRotationGivenAngularVelocity(const Quaternion &q,
                                              const V3D &angularVelocity,
                                              double dt) {
    double angularVelocityMagnitude = angularVelocity.norm(); // norm() returns square root of squared norm
    // avoid divide by close to zero...
    if (angularVelocityMagnitude > 1e-10) {
        Quaternion qnew;

        // TODO: Ex.1 Integration
        // implement quaternion update logic
        // q_p = rot(w, dt) * q

        // idea is to rotate unit quaternion by another unit quaternion defined by the direction of w.dot(dt)
        // Axis angle definition of unit quaternion:
        // q.w == cos(angle / 2) --> magniture of rotation
        // q.x == sin(angle / 2) --> direction of rotation (normalized)
        // q.y == sin(angle / 2) 
        // q.z == sin(angle / 2) 
        // where angle is defined as the product of the norm of w and dt.
        double w = cos(angularVelocityMagnitude*dt/2);
        double x = sin(angularVelocityMagnitude*dt/2)*angularVelocity[0]/angularVelocityMagnitude;
        double y = sin(angularVelocityMagnitude*dt/2)*angularVelocity[1]/angularVelocityMagnitude;
        double z = sin(angularVelocityMagnitude*dt/2)*angularVelocity[2]/angularVelocityMagnitude;
    
        qnew = Quaternion(w,x,y,z)*q;
        //std::cout << "qnew: " << qnew.w() << " " << qnew.x() << " " << qnew.y() << " " << qnew.z() << " " << qnew.norm()<< std::endl;

        return qnew;
    }
    return q;
}

/**
 * "base" class of our simulation world governed by the rigid-body dynamics
 */
class RBEngine {
public:
    // constructor
    RBEngine() {}

    // desctructor
    virtual ~RBEngine() {
        // we are going to delete every rigid body when simulation world is
        // destroyed.
        for (uint i = 0; i < rbs.size(); i++) delete rbs[i];
        rbs.clear();
        for (uint i = 0; i < springs.size(); i++) delete springs[i];
        springs.clear();
    }

    /**
     * add rigid body to simulation world.
     * note that we assume this is a cube block with approx. 0.24 x 0.24 x 0.24. 
     */
    RB *addRigidBodyToEngine() {
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.collision = false;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        return rbs.back();
    }

    /**
     * add rigid body with collision to simulation world.
     * note that we assume this is a sphere with radius = 0.1. 
     */
    RB *addCollidingRigidBodyToEngine() {
        double i = 0.4 * 100 * 0.1 * 0.1;
        rbs.push_back(new RB());
        // we use default mass = 100 kg
        rbs.back()->rbProps.mass = 100;
        rbs.back()->rbProps.setMOI(i, i, i, 0, 0, 0);
        rbs.back()->rbProps.collision = true;
        rbs.back()->rbProps.id = rbs.size() - 1;
        // add force and torque
        f_ext.push_back(V3D());
        tau_ext.push_back(V3D());
        return rbs.back();
    }

    /**
     * add spring to simulation world.
     */
    RBSpring *addSpringToEngine(RB *parent, RB *child, P3D pJPos, P3D cJPos) {
        springs.push_back(new RBSpring());
        springs.back()->parent = parent;
        springs.back()->child = child;
        // local position of attached point from parent/child frames
        springs.back()->pJPos = pJPos;
        springs.back()->cJPos = cJPos;
        // we use default spring constant = 2000;
        springs.back()->k = 10000;

        // default rest length is the distance between attaching points when
        // the spring is added.
        if (parent == nullptr) {
            // TODO: Ex.2-1
            // implement your logic for a spring which is attached to world
            //
            // Hint:
            // - you can get world coordinates of local coordinate P3D p by
            // rb->state.getWorldCoordinates(p).

            // Notes:
            // Idea is to define l0 as the distance between the two joint points. This is then used to define the force on the spring.
            // l0 = (pJPos - cJPos).norm()
            // but if pJPos is nullptr, then pJpos is defined as the position of the parent rigid body.
            // need to have pJPos and cJPos in the world frame.

            V3D pParent = V3D(pJPos); // I am assuming this is just (0,0,0)
            V3D pChild = V3D(child->state.getWorldCoordinates(cJPos));
            // std::cout<< "pParent: " << pParent[0] << " " << pParent[1] << " " << pParent[2] << std::endl;
            // std::cout<< "pChild: " << pChild[0] << " " << pChild[1] << " " << pChild[2] << std::endl;
            springs.back()->l0 = (pParent-pChild).norm();  // TODO: change this!
        } else {
            // TODO: Ex.2-2
            // implement your logic for a spring where both ends are attached
            // to rigid bodies
            //
            //
            // Hint:
            // - you can get world coordinates of local coordinate P3D p by
            // rb->state.getWorldCoordinates(p).

            V3D pParent = V3D(parent->state.getWorldCoordinates(pJPos));
            V3D pChild = V3D(child->state.getWorldCoordinates(cJPos));

            // std::cout<< "pParent: " << pParent[0] << " " << pParent[1] << " " << pParent[2] << std::endl;
            // std::cout<< "pChild: " << pChild[0] << " " << pChild[1] << " " << pChild[2] << std::endl;

            springs.back()->l0 = (pParent - pChild).norm();  // TODO: change this!
        }
        return springs.back();
    }

    /**
     * apply external force (no spring force, no gravity. Force comes from 
     * third sources) to rigid body.
     */
    void applyForceTo(RB *rb, const V3D &f, const P3D &p) {
        // add force only if rb is in rbs
        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i] == rb) {
                f_ext[i] += f;
                V3D r = rb->state.getWorldCoordinates(V3D(p));
                tau_ext[i] += r.cross(f);
            }
        }
    }

    /**
     * simulation stepping logic. advance one simulation timestep with dt.
     * the unit of dt is second.
     *
     * note that this function is "pure virtual" function. Actual implementation
     * of step function should be completed in the "derived" class.
     */
    virtual void step(double dt) = 0;

    /**
     * draw every rigid body belongs to world.
     */
    inline void draw(const gui::Shader &rbShader) {
        // draw moi boxes
        for (uint i = 0; i < this->rbs.size(); i++) {
            if (!this->rbs[i]->rbProps.fixed) {
                if (this->rbs[i]->rbProps.collision)
                    crl::RBRenderer::drawCollisionRB(this->rbs[i], rbShader);
                else
                    crl::RBRenderer::drawMOI(this->rbs[i], rbShader);
            }
        }

        // draw springs
        for (uint i = 0; i < this->springs.size(); i++) {
            P3D start, end;
            if (this->springs[i]->parent == nullptr) {
                start = this->springs[i]->pJPos;
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            } else {
                start = this->springs[i]->parent->state.getWorldCoordinates(
                    this->springs[i]->pJPos);
                end = this->springs[i]->child->state.getWorldCoordinates(
                    this->springs[i]->cJPos);
            }
            drawCylinder(start, end, 0.05, rbShader);
        }

        // and now coordinate frames
        if (showCoordFrame) {
            for (uint i = 0; i < this->rbs.size(); i++)
                crl::RBRenderer::drawCoordFrame(this->rbs[i], rbShader);
        }
    }

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    RB *getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint) {
        RB *selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbs.size(); i++) {
            if (rbs[i]->getRayIntersectionPoint(ray, tmpIntersectionPoint)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbs[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }
        return selectedRB;
    }

protected:
    void updateForceForGravity() {
        for (uint i = 0; i < rbs.size(); i++) {
            // force and torque by gravity
            f_ext[i] += rbs[i]->rbProps.mass * V3D(0, RBGlobals::g, 0);
        }
    }

    void updateForceAndTorqueForSprings() {
        // force and torque by springs
        for (RBSpring *spring : springs) {
            // TODO: Ex.2 Spring force
            // compute spring force f_spring = -kx and torque tau_spring and
            // add them to f_ext and tau_ext
            //
            // Hint:
            // - spring->l0 is the rest length and spring->k is the spring contant
            // - you can retrieve index of rb in this->rbs list from rb->rbProps.id

            if (spring->parent == nullptr) {
                // TODO: Ex.2-1
                // implement your logic for a spring which is attached to world

                // force
                V3D f(0, 0, 0);  // TODO: change this!

                // Notes:
                // f = -k * (l - l0) where l is the current distance between the two points
                // l0 is just the norm of the rest length, so need to consider direction separately

                V3D pParent = V3D(spring->pJPos);
                V3D pChild = V3D(spring->child->state.getWorldCoordinates(spring->cJPos));
                V3D l = pChild - pParent;
                double l_norm = l.norm();
                V3D l_direction = l.normalized();

                f = -1 * spring->k * abs(l_norm - spring->l0) * l_direction;
                
                f_ext[spring->child->rbProps.id] += f;
                
                // std::cout<< "pParent: " << pParent[0] << " " << pParent[1] << " " << pParent[2] << std::endl;
                // std::cout<< "pChild: " << pChild[0] << " " << pChild[1] << " " << pChild[2] << std::endl;
                // std::cout<< "l is " << l[0] << " " << l[1] << " " << l[2] << std::endl;
                // std::cout<< "l_norm is " << l_norm << std::endl;
                // std::cout<< "l_direction is " << l_direction[0] << " " << l_direction[1] << " " << l_direction[2] << std::endl;
                // std::cout << "l0 is " << spring->l0 << std::endl;
                // std::cout << "f is " << f[0] << " " << f[1] << " " << f[2] << std::endl;

                // torque

                V3D tau(0, 0, 0);  // TODO: change this!

                // Notes:
                // tau is just the cross product of the force and the position vector from child to force point.
                // Tricky part here is just understanding wtf the difference between getWorldCoordinates(P3D) and getWorldCoordinates(V3D) is.
                // The latter is used to get a local offset vector.
                tau = (spring->child->state.getWorldCoordinates(V3D(spring->cJPos))).cross(f);
                tau_ext[spring->child->rbProps.id] += tau;
            } else {
                // TODO: Ex.2-2
                // implement your logic for a spring where both ends are attached
                // to rigid bodies.

                // force
                V3D f(0, 0, 0);  // TODO: change this!
                V3D pParent = V3D(spring->parent->state.getWorldCoordinates(spring->pJPos));
                V3D pChild = V3D(spring->child->state.getWorldCoordinates(spring->cJPos));
                V3D l = pChild - pParent;
                double l_norm = l.norm();
                V3D l_direction = l.normalized();

                f = -1 * spring->k * abs(l_norm - spring->l0) * l_direction;

                f_ext[spring->parent->rbProps.id] -= f;
                f_ext[spring->child->rbProps.id] += f;

                // torque
                V3D tau1(0, 0, 0);  // TODO: change this!
                V3D tau2(0, 0, 0);  // TODO: change this!
                tau1 = (spring->child->state.getWorldCoordinates(V3D(spring->cJPos))).cross(f);
                tau2 = - (spring->child->state.getWorldCoordinates(V3D(spring->pJPos))).cross(f);

                tau_ext[spring->parent->rbProps.id] += tau1;
                tau_ext[spring->child->rbProps.id] += tau2;
            }
        }
    }

    void updateVelocityAfterCollision(RB *rb) const {
        // TODO: Ex.4 Impulse-based Collisions
        // we will simulate collisions between a spherical rigidbody and
        // the ground plane. implement impulse-based collisions here. use
        // coefficient of restituation "epsilon". (it's a member variable
        // of this class).
        // we only implement collisions between ground and spheres.
        //
        // Steps:
        // 0. read the material "ImpulseBasedCollisions" on CMM21 website
        // carefully.
        // 1. compute impulse
        // 2. update linear and angular velocity with an impulse
        //
        // Hint:
        // - the radius of the sphere is 0.1 m
        // - detect collision if 1) the y coordinate of the point at the
        // bottom of the sphere < 0 and 2) the y component of linear
        // velocity of the point at the botton < 0.
        // - we will assume that a collision only happens at the bottom
        // points.
        // - we will assume there's only one contact between a sphere
        // and the ground

        // Notes:
        // impulse and change in velocity: F *dt = m * dv --> v' = v + impulse/ m.
        // restitutional impulse: u'_rel = - epsilon * u_rel
        // K_T = identity / m - r_cross * I_inv * r_cross, K is only for the ball and not ground since ground is a static infinite mass object. 
        
        P3D point = rb->state.getWorldCoordinates(P3D(0.0,-0.1,0.0));
        V3D point_velocity = rb->state.getVelocityForPoint_local(P3D(0.0,-0.1,0.0));

        bool collisionDetected = false;  // TODO: change this!

        if (point[1] < 0 ) { collisionDetected = true; }
        
        if (collisionDetected) {
            V3D impulse(0, 0, 0);
            Matrix3x3 I = rb->rbProps.MOI_local;
            P3D position = rb->state.pos;
            Matrix3x3 point_screw;
            point_screw <<  0, -position[2], position[1], position[2], 0, -position[0], -position[1], position[0], 0;
            Matrix3x3 K =(Matrix::Identity(3, 3) * 1.0 / rb->rbProps.mass) -(point_screw *(I.inverse().eval() * point_screw));

            V3D U_relative = rb->state.velocity + rb->state.angularVelocity.cross(V3D(position));

            V3D normal = V3D(0.0, 1.0, 0.0);

            if (frictionalCollision) {
                // TODO: compute infinite friction collision impulse
                impulse = V3D(K.inverse() * (-U_relative - (double)eps * (U_relative.dot(normal)) * normal));

            } else {
                // TODO: compute frictionless collision impulse
                double impulse_normal = -(1+eps)* U_relative.dot(normal)/(normal.transpose() * K * normal);
                impulse = impulse_normal*normal;
            }
             
            rb->state.velocity +=  impulse / rb->rbProps.mass;
            rb->state.angularVelocity += I.inverse()* (V3D(position).cross(impulse));  

            collisionDetected=false; 
        }
        
    }

public:
    // this is a list of all rigid bodies and springs belong to the world.
    std::vector<RB *> rbs;
    std::vector<RBSpring *> springs;

    // coefficients
    float eps = 0.7;  // restitution

    // drawing flags
    bool showCoordFrame = true;

    // options
    bool simulateCollisions = false;
    bool frictionalCollision = false;

protected:
    // list of force and tau applied to rigid bodies
    std::vector<V3D> f_ext;
    std::vector<V3D> tau_ext;
};
}  // namespace crl