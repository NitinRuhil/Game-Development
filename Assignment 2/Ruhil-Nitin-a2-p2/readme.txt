Name: Nitin Ruhil
Student Number: V01016220
Course: CSC 486A
Assignment: A2 – Part 2
Project: Softbody Simulation System

------------------------------------------------------------
What I Completed
------------------------------------------------------------

1. Particle Initialization
- The script reads the mesh from the MeshFilter.
- One particle is created for every vertex in the mesh.
- Vertices are converted from local space to world space.
- Each particle stores its position, velocity, mass, list of springs, contact spring, and accumulated forces.

2. Spring Initialization
- Every particle is connected to every other particle once (i < j approach).
- Each spring stores ks, kd, rest length, and the index of the connected particle.
- Rest lengths are calculated using the original positions of the vertices.
- No duplicate springs are created.

3. Ground Plane Setup
- The ground plane position and normal are set using a Transform from the scene.
- The plane normal uses groundPlaneTransform.up.
- I assign the ground object in the Unity Inspector.

4. Contact Forces (Ground Penalty Spring)
- When a particle goes below the plane, a contact spring is created.
- The attach point is the closest point on the plane.
- The contact force uses:
  F = -ks * penetration * normal - kd * velocity
- When the particle moves above the plane again, the contact spring is removed.

5. Internal Spring Forces
- Hooke’s Law is used for the spring forces.
- Damping is applied along the spring direction.
- Forces apply equal and opposite values to the two connected particles.
- Prevents double-counting and keeps the system stable.

6. Gravity
- Gravity force is added when useGravity is enabled.
- Gravity is set to (0, -9.8, 0) by default.

7. Symplectic Euler Integration
- I update velocity first: v = v + a * dt
- Then update position: x = x + v * dt
- This method is more stable for spring systems.

8. Mesh Update
- After integration, particle positions are converted back to local coordinates.
- Mesh vertices are updated every frame.
- Bounds and normals are recalculated.

9. Inspector Options
The script exposes the following settings:
- groundPlaneTransform
- handlePlaneCollisions
- particleMass
- useGravity
- gravity
- defaultSpringKS
- defaultSpringKD
- contactSpringKS
- contactSpringKD
- debugRender

10. Test Case Parameter Settings
For each of the three provided cubes:
Blue Cube:
  ks = 200
  kd = 0
Red Cube:
  ks = 80
  kd = 0.8
Green Cube:
  ks = 45
  kd = 0.2

Other common settings:
  particleMass = 1
  useGravity = true
  handlePlaneCollisions = true

------------------------------------------------------------
What Works
------------------------------------------------------------
- Soft cubes fall, stretch, squash, and settle correctly.
- All vertices update smoothly.
- Spring forces and damping behave correctly.
- Ground collision uses penalty spring forces.
- No Unity physics system (no Rigidbody, no colliders).
- Debug mode correctly draws spring connections and forces.

------------------------------------------------------------
How to Test
------------------------------------------------------------
1. Open the Unity project.
2. Press Play.
3. The cubes fall, squash on the ground, and settle.
4. Turning on debugRender shows the springs visually.
5. Changing ks and kd changes how soft or stiff the cube feels.

------------------------------------------------------------
Files Included
------------------------------------------------------------
- BParticleSimMesh.cs
- Entire Unity project folder (following .gitignore rules)
- readme.txt (this file)