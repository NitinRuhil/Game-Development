Assignment 3 – Boids Simulation and Path Following
Name:Nitin Ruhil
Student ID: V01016220

-----------------------------------

1. What this project does
This project creates a swarm of boids (200 bugs) that move using flocking rules:

    - Separation (boids move away if too close)
    - Alignment (boids match neighbour direction)
    - Cohesion (boids move toward group center)
    - Wander (when no neighbours are seen)
    - Obstacle avoidance
    - World boundary avoidance
    - Special path-following rule for Boid 0 using Unity NavMesh

Boid 0 moves to a goal when the App script calls SetGoal() after pressing Space.

-----------------------------------

2. What works (completed parts)

Boid Initialization
    - All 200 boids spawn inside the initialization radius.
    - Each boid gets a random forward direction.
    - Velocities and forces start correctly.

Simulation Loop
    - Forces reset each frame.
    - All boids update every FixedUpdate.
    - Symplectic Euler integration is implemented:
    - velocity = velocity + acceleration * dt
    - position = position + velocity * dt

Neighbour System
    - Neighbours detected using distance and dot product (field of view).

Flocking Rules
    - Separation rule implemented.
    - Alignment rule implemented.
    - Cohesion rule implemented.
    - Wander rule when no neighbours exist.

Obstacle / Boundary Rules
    - Physics.OverlapSphere detects nearby obstacles.
    - ClosestPoint is used to compute escape direction.
    - World boundaries act like obstacles.
    - Y-position is clamped between worldMinY and worldMaxY.

Total Force
    - All forces are accumulated correctly.

Boid Zero Path Following
    - SetGoal uses NavMesh.SamplePosition.
    - NavMesh.CalculatePath computes a valid path.
    - Boid 0 moves corner-by-corner.
    - When path ends, state resets correctly.

Boid Mesh Updates
    - Visual meshes follow positions and rotate toward velocity.

Default Values
    - Original testcase values are preserved.

-----------------------------------

4. How to use

    1. Enter Play mode in Unity.
    2. Make sure Swarm object is assigned to the App script in Inspector.
    3. Press Space to send boid 0 to the target.
    4. Watch the flocking behaviour and path-following.

-----------------------------------

5. Files included

readme.txt
Unity project files (following .gitignore rules)

-----------------------------------

6. Notes
    - Debug lines show forces and path for boid zero during play mode.
    - Behaviour is stable.