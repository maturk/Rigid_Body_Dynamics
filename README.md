In this assignment, I implemented rigid body dynamics simulator for projectile motion, spring like system, and a bouncing ball with contact constraints. 

# Rigid bodies 

### Ex.1 Projectile Motion

A rigid body projectile motion simulator was created using both explicit and symplectic Euler integration schemes. Below is a short demo of both Euler integrators.
![explicitAndSymplecticIntegrators](https://user-images.githubusercontent.com/30566358/173825749-7f1694d6-67ee-4edb-bb45-efa60b2c6baa.gif)

![equation: newton-euler equation](imgs/eq-newton-euler.png)

where **v**, **w**, **p**, **R** are the linear velocity of the center of mass of a rigidbody, the angular velocity, the position and the orientation (3x3 rotational matrix) respectively (note that they are all expressed in world frame!) *m* and **I** denote mass and moment of inertia (expressed in world frame) of the rigid body. We want to find **v**, **w**, **p**, **R** at time *t*, given profile of external force **F** and torque **t**. 

If we use an unit quaternion for orientation instead of a rotational matrix and discretize this equation, we get: 

![equation: discretized newton-euler equation](imgs/eq-discrete-velocity.png)

![equation: discretized newton-euler equation](imgs/eq-discrete-pose.png)

where *h* is a timestep of the simulation. 

### Ex.2 Springs (baseline - 20%)

A spring simulator is also created. A spring is attached to a rigidbody as an external force. 

![spring](https://user-images.githubusercontent.com/30566358/173827056-a9b83070-cd00-47ea-a866-97c6d44f5ac2.gif)

### Ex.3 Impulse-based Collisions (advanced)

Impulse based collision is simulated with the following assumptions:

- a collision only happens at the bottom of a sphere 
- there's only one contact between a sphere and the ground
- the radius of the sphere is 0.1 m

Both **frictionless** collision and **infinite friction** collision are implemented.  

![figure: successful implementation of ex4](imgs/ex4.gif)
